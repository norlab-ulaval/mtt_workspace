#!/usr/bin/env python3
"""Save a local source snapshot, Git state and hashes without contacting remotes.

Includes checked-out tracked files from recursive submodules, non-ignored
maintained source, ignored research helpers under scripts/research, and root
Python helpers. Excludes ignored documentation/assistant notes as well as
datasets, caches, virtual environments, .env and Git object stores.
This is a source snapshot, not a backup of bags, Docker images or Git history.
"""

import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile


ROOT = Path(__file__).resolve().parents[1]
LOCAL_ROOTS = ('scripts', 'docs', 'demos', 'research', 'tests', 'documentations', '.github')
LOCAL_SUFFIXES = {'.py', '.sh', '.md', '.yaml', '.yml', '.json', '.toml', '.txt', '.xacro'}


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args])


def repositories(root):
    # foreach only visits initialized submodules. Missing ones are visible in
    # submodule_status; do not silently present them as included source trees.
    paths = git(root, 'submodule', 'foreach', '--quiet', '--recursive',
                'printf "%s\\n" "$displaypath"').decode().splitlines()
    return [Path('.'), *(Path(path) for path in paths)]


def collect(root):
    paths = set()
    repos = []
    for relative in repositories(root):
        repo = root / relative
        listing = ['ls-files', '-z', '--cached']
        if relative != Path('.'):
            # A parent's file list contains only gitlinks, never new child
            # files. Preserve local tests/source inside each child as well.
            listing += ['--others', '--exclude-standard']
        files = git(repo, *listing).split(b'\0')
        for name in files:
            if name:
                path = relative / os.fsdecode(name)
                if path.name.startswith('.env') or any(part in {'.git', '.ssh'} for part in path.parts):
                    continue
                if (root / path).is_file() or (root / path).is_symlink():
                    paths.add(path)
        repos.append({
            'path': str(relative), 'head': git(repo, 'rev-parse', 'HEAD').decode().strip(),
            'status': git(repo, 'status', '--porcelain=v1', '--untracked-files=all').decode(),
            'branches_containing_head': git(repo, 'branch', '-r', '--contains', 'HEAD').decode().splitlines(),
            'note': 'Remote refs are local observations; no remote availability check was made.',
        })
    # Include new maintained files even when they have no suffix (shell tools).
    for name in git(root, 'ls-files', '--others', '--exclude-standard', '-z').split(b'\0'):
        if not name:
            continue
        path = Path(os.fsdecode(name))
        if path.parts[0] in LOCAL_ROOTS and not path.name.startswith('.env'):
            paths.add(path)
    # Recover local research implementations too, but do not package ignored
    # planning notes or local assistant instructions as release documentation.
    for name in ('scripts', 'research'):
        for directory, subdirs, filenames in os.walk(root / name, followlinks=False):
            subdirs[:] = [d for d in subdirs if not d.startswith('.') and d not in
                          {'__pycache__', 'build', 'install', 'log', 'data', 'results', 'node_modules'}]
            for filename in filenames:
                path = Path(directory) / filename
                if not filename.startswith('.') and path.suffix in LOCAL_SUFFIXES:
                    paths.add(path.relative_to(root))
    paths.update(path.relative_to(root) for path in root.glob('*.py'))
    return sorted(paths), repos


def write_snapshot(root, output):
    output.mkdir(parents=True, exist_ok=False)
    paths, repos = collect(root)
    manifest = {
        'scope': __doc__, 'repositories': repos,
        'submodule_status': git(root, 'submodule', 'status', '--recursive').decode(),
        'files': [],
    }
    archive = output / 'sources.tar.gz'
    with tarfile.open(archive, 'w:gz', compresslevel=1) as tar:
        for path in paths:
            absolute = root / path
            if absolute.is_symlink():
                manifest['files'].append({'path': str(path), 'symlink': os.readlink(absolute)})
                tar.add(absolute, arcname=str(Path('mtt_workspace') / path), recursive=False)
                continue
            data = absolute.read_bytes()
            manifest['files'].append({
                'path': str(path), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()})
            info = tar.gettarinfo(str(absolute), arcname=str(Path('mtt_workspace') / path))
            tar.addfile(info, io.BytesIO(data))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    for index, repo in enumerate(repos):
        (output / f'repo-{index:02d}.patch').write_bytes(git(root / repo['path'], 'diff', '--binary', 'HEAD'))
    checksums = []
    for artifact in sorted(output.iterdir()):
        with artifact.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        checksums.append(f'{digest}  {artifact.name}\n')
    (output / 'SHA256SUMS').write_text(''.join(checksums))
    print(f'{output}: {len(paths)} files; {len(repos)} repositories; {archive.stat().st_size} compressed bytes')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New directory; refuses an existing one')
    args = parser.parse_args()
    write_snapshot(ROOT, args.output.resolve())


if __name__ == '__main__':
    main()
