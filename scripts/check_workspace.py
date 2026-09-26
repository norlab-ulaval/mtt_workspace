#!/usr/bin/env python3
"""Check source files without importing project code or connecting to ROS.

Uses Git's tracked and non-ignored file lists so datasets, virtual environments,
vendor repositories and generated outputs are not traversed. Untracked source
is checked during development; `verify --release` separately requires commits.
"""

import argparse
import ast
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = {'scripts', 'demos', 'docs', 'docker', 'tests', 'research', '.github'}
CONFLICT = re.compile(r'^(?:<{7,} |>{7,} |={7,}$)', re.MULTILINE)


def literal_path_parts(node):
    """Constant suffix of a Path division expression; never evaluate code."""
    if (isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div)
            and isinstance(node.right, ast.Constant) and isinstance(node.right.value, str)):
        return (*literal_path_parts(node.left), node.right.value)
    return ()


def source_files(root):
    result = subprocess.run(
        ['git', '-C', str(root), 'ls-files', '-z', '--cached', '--others',
         '--exclude-standard'], check=True, stdout=subprocess.PIPE)
    return sorted({
        Path(name.decode()) for name in result.stdout.split(b'\0') if name
        if len(Path(name.decode()).parts) == 1
        or Path(name.decode()).parts[0] in SOURCE_ROOTS
    })


def check(root):
    files = source_files(root)
    errors = []
    python_count = 0
    local_modules = {
        p.stem: p.relative_to(root)
        for folder in ('scripts', 'scripts/mtt_motion_model')
        for p in (root / folder).glob('*.py')
    }
    for relative in files:
        path = root / relative
        if not path.is_file() or path.is_symlink():
            continue
        if path.suffix not in {'.py', '.md', '.sh', '.yaml', '.yml', '.xml', '.json', ''}:
            continue
        if path.stat().st_size > 2_000_000:
            continue
        try:
            content = path.read_text()
        except UnicodeDecodeError:
            continue
        if CONFLICT.search(content):
            errors.append(f'{relative}: unresolved merge conflict marker')
        if relative.suffix == '.py':
            python_count += 1
            try:
                tree = ast.parse(content, filename=str(relative))
            except SyntaxError as exc:
                errors.append(f'{relative}:{exc.lineno}: {exc.msg}')
                continue
            for node in ast.walk(tree):
                parts = literal_path_parts(node)
                if parts and parts[0] in {'scripts', 'demos'} and parts[-1].endswith('.py'):
                    dependency = Path(*parts)
                    if dependency not in files or not (root / dependency).is_file():
                        errors.append(f'{relative}: missing subprocess helper {dependency}')
                names = []
                if isinstance(node, ast.Import):
                    names = [alias.name.split('.')[0] for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module and not node.level:
                    names = [node.module.split('.')[0]]
                    # A local package directory can survive while its Python
                    # module was deleted/ignored (for example scripts/lib).
                    parts = node.module.split('.')
                    if len(parts) > 1 and (root / 'scripts' / parts[0]).is_dir():
                        module = Path('scripts').joinpath(*parts).with_suffix('.py')
                        package = Path('scripts').joinpath(*parts) / '__init__.py'
                        if module not in files and package not in files:
                            errors.append(
                                f'{relative}:{node.lineno}: missing local module {node.module}')
                for name in names:
                    dependency = local_modules.get(name)
                    if dependency is not None and dependency not in files:
                        errors.append(
                            f'{relative}:{node.lineno}: local import {dependency} is ignored by Git')
        if relative.suffix == '.md':
            for match in re.finditer(r'\[[^\]]*\]\(([^\s)]+)(?:\s+[^)]*)?\)', content):
                target = unquote(match[1].split('#')[0])
                if (not target or target.startswith('/') or
                        re.match(r'[\w+.-]+:', target)):
                    continue
                if not (path.parent / target).exists():
                    errors.append(f'{relative}: missing linked file {target}')
        if relative.name == 'compose.yaml':
            for script in re.findall(r'\$\{WORKSPACE\}/(scripts/[\w/.-]+\.py)', content):
                if Path(script) not in files or not (root / script).is_file():
                    errors.append(f'{relative}: script absent from source file list: {script}')
    return sorted(set(errors)), python_count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT, help=argparse.SUPPRESS)
    args = parser.parse_args()
    errors, count = check(args.root.resolve())
    for error in errors:
        print(f'[source] FAIL {error}')
    print(f'[source] {count} Python files parsed; {len(errors)} source/documentation errors')
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
