"""Restore byte-identical source databases from bundled gzip chunks using stdlib."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path

class ChunkReader(io.RawIOBase):
    def __init__(self, paths):
        self.paths = iter(paths)
        self.current = None

    def readable(self):
        return True

    def readinto(self, buffer):
        while True:
            if self.current is None:
                path = next(self.paths, None)
                if path is None:
                    return 0
                self.current = path.open('rb')
            count = self.current.readinto(buffer)
            if count:
                return count
            self.current.close()
            self.current = None

    def close(self):
        if self.current is not None:
            self.current.close()
        super().close()

def restore(root, dataset):
    manifest_path = root / 'archives.json'
    if not manifest_path.is_file():
        return False
    record = json.loads(manifest_path.read_text())['datasets'].get(dataset)
    if record is None:
        return False
    paths = []
    for part in record['parts']:
        path = root / part['file']
        if not path.is_file() or path.stat().st_size != part['bytes']:
            raise RuntimeError(f'Missing data chunk or unresolved LFS pointer: {path}. Run git lfs pull.')
        h = hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
                h.update(block)
        if h.hexdigest() != part['sha256']:
            raise RuntimeError(f'Data chunk checksum mismatch: {path}')
        paths.append(path)
    target = root / record['database']
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix(target.suffix + '.partial')
    print(f'[{dataset}] Restoring original database from bundled data', flush=True)
    h = hashlib.sha256()
    size = 0
    try:
        with ChunkReader(paths) as raw, io.BufferedReader(raw) as stream, gzip.GzipFile(fileobj=stream, mode='rb') as source, partial.open('wb') as out:
            for block in iter(lambda: source.read(8 * 1024 * 1024), b''):
                h.update(block)
                size += len(block)
                out.write(block)
        if size != record['bytes'] or h.hexdigest() != record['sha256']:
            raise RuntimeError(f'Restored database checksum mismatch: {dataset}')
        partial.replace(target)
    except BaseException:
        partial.unlink(missing_ok=True)
        raise
    print(f'[{dataset}] Original database SHA256 verified', flush=True)
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=Path(__file__).resolve().parents[1] / 'data')
    parser.add_argument('--dataset', choices=['both', 'fin301', 'Fin301-11051740'], default='both')
    args = parser.parse_args()
    for name in (['fin301', 'Fin301-11051740'] if args.dataset == 'both' else [args.dataset]):
        if not restore(args.data_root, name):
            parser.error(f'Bundled archive unavailable for {name}')
