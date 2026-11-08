"""Portable runner for the final mapping workflow on either recording."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=['both', 'fin301', 'Fin301-11051740'], default='both')
    parser.add_argument('--data-root', type=Path, default=ROOT / 'data')
    parser.add_argument('--work-root', type=Path, default=ROOT / 'work')
    parser.add_argument('--results-root', type=Path, default=ROOT / 'results')
    parser.add_argument('--cartographer-prefix', type=Path, help='Optional custom Cartographer ROS installation prefix')
    parser.add_argument('--reference-measurements', type=Path, help='JSON object keyed by dataset, containing independent physical distance checks')
    parser.add_argument('--rebuild-backend', action='store_true')
    args = parser.parse_args()
    datasets = json.loads((ROOT / 'config/datasets.json').read_text())
    names = list(datasets) if args.dataset == 'both' else [args.dataset]
    # Check every selected bag before starting a long reconstruction.
    for name in names:
        bag = (args.data_root / datasets[name]['bag']).resolve()
        if not bag.is_file():
            from restore_data import restore
            restore(args.data_root.resolve(), name)
        if not bag.is_file():
            parser.error(f'Missing recording: {bag}. Supply --data-root with both recording folders.')
    env = os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    if args.cartographer_prefix:
        prefix = args.cartographer_prefix.resolve()
        runtime_root = prefix.parents[2]
        env['AMENT_PREFIX_PATH'] = str(prefix) + os.pathsep + env.get('AMENT_PREFIX_PATH', '')
        extras = [prefix / 'lib', runtime_root / 'usr/lib', runtime_root / 'usr/lib/x86_64-linux-gnu']
        env['LD_LIBRARY_PATH'] = os.pathsep.join(map(str, extras)) + os.pathsep + env.get('LD_LIBRARY_PATH', '')
        python_dir = prefix / f'local/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages'
        env['PYTHONPATH'] = str(python_dir) + os.pathsep + env.get('PYTHONPATH', '')
    else:
        try:
            from ament_index_python.packages import get_package_prefix
            prefix = Path(get_package_prefix('cartographer_ros'))
        except (ImportError, LookupError) as exc:
            parser.error(f'Source ROS and Cartographer setup files first, or supply --cartographer-prefix: {exc}')
    commands = {name: prefix / 'lib/cartographer_ros' / name for name in
                ['cartographer_offline_node', 'cartographer_pbstream_to_ros_map', 'cartographer_node']}
    for command in commands.values():
        if not command.is_file():
            parser.error(f'Required Cartographer executable missing: {command}')

    for name in names:
        work = (args.work_root / name).resolve()
        work.mkdir(parents=True, exist_ok=True)
        bag = (args.data_root / datasets[name]['bag']).resolve()
        env.update(MAP_DATASET=name, MAP_BAG_PATH=str(bag), MAP_WORK_DIR=str(work),
                   MAP_RESULTS_DIR=str((args.results_root / name).resolve()),
                   ROS_DOMAIN_ID=str(183 + list(datasets).index(name)))
        config_hash = hashlib.sha256(b''.join(p.read_bytes() for p in sorted((ROOT / 'config/cartographer').glob('*.lua')))).hexdigest()
        signature = {'source_sha256': digest(bag), 'config_sha256': config_hash, 'format_version': 2}
        cache_key = work / 'input_signature.json'
        stale = not cache_key.exists() or json.loads(cache_key.read_text()) != signature
        if stale:
            for relative in ['backend', 'baseline', 'raw_scans', 'camera', 'camera_scans', 'baseline_config']:
                path = work / relative
                if path.exists():
                    shutil.rmtree(path)
        backend = work / 'backend'
        backend.mkdir(exist_ok=True)
        logs = work / 'logs'
        logs.mkdir(exist_ok=True)

        def run(command, label):
            print(f'[{name}] {label}', flush=True)
            with (logs / (label + '.log')).open('w') as output:
                subprocess.run(list(map(str, command)), env=env, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, check=True)

        def stage(script):
            run([sys.executable, ROOT / 'src' / (script + '.py')], script)

        if not (work / 'raw_scans/metadata.yaml').is_file():
            if (work / 'raw_scans').exists():
                shutil.rmtree(work / 'raw_scans')
            stage('prepare_scans')
        cache_key.write_text(json.dumps(signature, indent=2))
        baseline = work / 'baseline'
        baseline.mkdir(exist_ok=True)
        baseline_config = work / 'baseline_config'
        shutil.copytree(ROOT / 'config/cartographer', baseline_config, dirs_exist_ok=True)
        config_file = baseline_config / 'fin301.lua'
        config_file.write_text(config_file.read_text().replace('use_landmarks = true', 'use_landmarks = false'))

        def query_state(folder, configuration):
            with (logs / ('query_' + folder.name + '.log')).open('w') as output:
                node = subprocess.Popen([str(commands['cartographer_node']),
                    '-configuration_directory=' + str(configuration), '-configuration_basename=fin301.lua',
                    '-load_state_filename=' + str(folder / 'map.pbstream'),
                    '-load_frozen_state=true', '-start_trajectory_with_default_topics=false'],
                    env=env, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT)
                try:
                    env['MAP_QUERY_DIR'] = str(folder)
                    stage('query_poses')
                finally:
                    node.terminate()
                    try:
                        node.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        node.kill();node.wait()

        if args.rebuild_backend or not (baseline / 'optimized_poses.csv').is_file():
            run([commands['cartographer_offline_node'], '-configuration_directory=' + str(baseline_config),
                 '-configuration_basenames=fin301.lua', '-bag_filenames=' + str(work / 'raw_scans'),
                 '-use_bag_transforms=false', '-save_state_filename=' + str(baseline / 'map.pbstream')], 'baseline')
            query_state(baseline, baseline_config)
        if args.rebuild_backend or not (work / 'camera/revisits.json').is_file() or not (work / 'camera_scans/metadata.yaml').is_file():
            stage('camera_revisits')
        rebuilt_final = args.rebuild_backend or not all((backend / n).is_file() for n in ['native_map.pgm', 'native_map.yaml'])
        if rebuilt_final:
            run([commands['cartographer_offline_node'], '-configuration_directory=' + str(ROOT / 'config/cartographer'),
                 '-configuration_basenames=fin301.lua', '-bag_filenames=' + str(work / 'camera_scans'),
                 '-use_bag_transforms=false', '-save_state_filename=' + str(backend / 'map.pbstream')], 'camera_fused')
            run([commands['cartographer_pbstream_to_ros_map'], '-pbstream_filename=' + str(backend / 'map.pbstream'),
                 '-map_filestem=' + str(backend / 'native_map'), '-resolution=.025'], 'native_occupancy')
        if rebuilt_final or not (backend / 'optimized_poses.csv').is_file():
            query_state(backend, ROOT / 'config/cartographer')
        if args.reference_measurements:
            references = json.loads(args.reference_measurements.read_text())
            reference_file = work / 'reference_measurements.json'
            reference_file.write_text(json.dumps(references.get(name, [])))
            env['MAP_REFERENCE_MEASUREMENTS'] = str(reference_file)
        else:
            env.pop('MAP_REFERENCE_MEASUREMENTS', None)
        stage('export_results')
        stage('evaluate_mapping')
        print(f'[{name}] Complete: {env["MAP_RESULTS_DIR"]}', flush=True)

if __name__ == '__main__':
    main()
