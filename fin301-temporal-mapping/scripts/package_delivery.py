"""Package one dataset's verified outputs without raw recordings or caches.

Example: python scripts/package_delivery.py --dataset fin301
The default legacy layout remains available through --dataset legacy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import posixpath
import re
import zipfile


ROOT = Path(__file__).resolve().parents[1]


def dataset_prefix(dataset):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', dataset):
        raise ValueError('dataset must be a simple name containing letters, numbers, _ or -')
    return Path('outputs') if dataset == 'legacy' else Path('outputs') / dataset


def collect_files(root, dataset='legacy'):
    """Return archive-name -> source-path for an explicitly scoped delivery."""
    root = Path(root).resolve()
    prefix = dataset_prefix(dataset)
    files = {}

    def add(path, archive_name=None):
        path = Path(path)
        if not path.is_file() or path.is_symlink():
            return
        if any(part in path.parts for part in ('__pycache__', 'plot_cache', '.mplconfig')) or path.suffix in ('.pyc', '.pyo'):
            return
        name = archive_name or path.relative_to(root).as_posix()
        files[name] = path

    for name in ['setup.py', 'setup.cfg', 'package.xml', 'pytest.ini',
                 'requirements.txt', 'requirements-lock.txt', '.gitignore']:
        add(root / name)
    if dataset == 'legacy':
        add(root / 'README.md')
        add(root / 'ROS_RUNTIME.md')
        doc_names = ['references.md', 'individual_contribution_template.md',
                     'report_notes_zh.md', 'implementation_plan.md', 'delivery_checklist.md']
    else:
        readme = root / 'docs' / f'{dataset}_readme.md'
        if not readme.is_file():
            raise FileNotFoundError(f'Dataset delivery README is required: {readme}')
        add(readme, 'README.md')
        doc_names = ['references.md', 'individual_contribution_template.md',
                     f'{dataset}_readme.md', f'report_notes_{dataset}_zh.md',
                     f'localization_validation_{dataset}.md']
    for name in doc_names:
        add(root / 'docs' / name)
    for directory in [Path(name) for name in ('indoor_mapping', 'scripts', 'tests', 'config', 'launch', 'resource')] + [prefix / name for name in ('result', 'report', 'verification')]:
        for path in (root / directory).rglob('*'):
            add(path)
    if dataset != 'legacy':
        # Keep the finite, documented comparison artifacts and baseline. Raw
        # recordings, image caches, ROS build trees and unrelated runs stay out.
        for name in ('baseline', 'accuracy_experiment', 'accuracy_partial_overlap'):
            for path in (root / prefix / name).rglob('*'):
                add(path)
    for name in ['audit.md', 'audit.json', 'calibration.json', 'rosout.json']:
        add(root / prefix / 'audit' / name)
    return dict(sorted(files.items()))


def verify_archive(archive):
    """Check ZIP CRCs and every manifest size/hash; reject unlisted members."""
    with zipfile.ZipFile(archive) as stream:
        names = stream.namelist()
        if len(names) != len(set(names)):
            raise ValueError('archive contains duplicate member names')
        if stream.testzip() is not None:
            raise ValueError('archive CRC integrity check failed')
        manifest = json.loads(stream.read('DELIVERY_MANIFEST.json'))
        if set(names) != set(manifest) | {'DELIVERY_MANIFEST.json'}:
            raise ValueError('archive contents disagree with the manifest')
        for name, expected in manifest.items():
            path = PurePosixPath(name)
            if path.is_absolute() or '..' in path.parts:
                raise ValueError(f'unsafe archive path: {name}')
            content = stream.read(name)
            if len(content) != expected['bytes']:
                raise ValueError(f'manifest size mismatch: {name}')
            if hashlib.sha256(content).hexdigest() != expected['sha256']:
                raise ValueError(f'manifest hash mismatch: {name}')
    return {'files': len(manifest), 'zip_integrity': 'passed', 'manifest_hashes': 'passed'}


def write_archive(files, archive):
    """Write bytes and their manifest from the same snapshots, then verify."""
    archive = Path(archive)
    archive.parent.mkdir(parents=True, exist_ok=True)
    manifest = {}
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as stream:
        for name, path in sorted(files.items()):
            content = Path(path).read_bytes()
            if name == 'README.md' and Path(path).parent.name == 'docs':
                # The same source is also stored unchanged under docs/. Only
                # the root copy needs its relative Markdown links rebased.
                def rebase(match):
                    target = match.group(2)
                    if target.startswith(('/', '#')) or ':' in target:
                        return match.group(0)
                    return match.group(1) + posixpath.normpath('docs/' + target) + match.group(3)
                content = re.sub(r'(\[[^\]]*\]\()([^\s)]+)(\))', rebase, content.decode('utf-8')).encode('utf-8')
            manifest[name] = {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest()}
            stream.writestr(name, content)
        stream.writestr('DELIVERY_MANIFEST.json', json.dumps(manifest, indent=2))
    return verify_archive(archive)


def build_archive(root=ROOT, dataset='legacy'):
    root = Path(root).resolve()
    prefix = dataset_prefix(dataset)
    verification = root / prefix / 'verification' / 'artifact_checks.json'
    checks = json.loads(verification.read_text(encoding='utf-8'))
    if not checks.get('passed') or not checks.get('checked_file_sha256'):
        raise ValueError('Run verify_delivery.py for this dataset before packaging')
    if checks.get('dataset') != dataset:
        raise ValueError('Artifact verification belongs to a different dataset')
    required_verified = [prefix / 'result' / name for name in ('map.pcd', 'metrics.json', 'trajectory.csv')]
    required_verified += [prefix / 'report' / name for name in ('group_technical_report.pdf', 'report_manifest.json')]
    if any(path.as_posix() not in checks['checked_file_sha256'] for path in required_verified):
        raise ValueError('Dataset verification does not cover all primary delivery artifacts')
    for name, expected in checks['checked_file_sha256'].items():
        path = (root / name.replace('\\', '/')).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError(f'Verified source is missing or outside the project: {name}')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'File changed after artifact verification: {name}')
    files = collect_files(root, dataset)
    for relative in [prefix / 'result' / 'map.pcd', prefix / 'report' / 'group_technical_report.pdf',
                     prefix / 'report' / 'report_manifest.json', prefix / 'verification' / 'artifact_checks.json',
                     *[prefix / 'audit' / name for name in ('audit.md', 'audit.json', 'calibration.json', 'rosout.json')]]:
        if relative.as_posix() not in files:
            raise FileNotFoundError(f'Required delivery file missing: {relative}')
    archive = root / 'deliverables' / ('Project_2_2_deliverables.zip' if dataset == 'legacy' else f'Project_2_2_{dataset}.zip')
    integrity = write_archive(files, archive)
    return {'archive': str(archive), 'dataset': dataset, 'bytes': archive.stat().st_size, **integrity}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', default='legacy', help='Dataset folder under outputs; legacy uses outputs/result')
    args = parser.parse_args()
    print(json.dumps(build_archive(dataset=args.dataset), indent=2))


if __name__ == '__main__':
    main()
