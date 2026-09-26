"""Offline regression tests: real local Git repositories, no Docker or ROS."""

import importlib.util
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('check_workspace', ROOT / 'scripts/check_workspace.py')
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)
SNAPSHOT_SPEC = importlib.util.spec_from_file_location('handover_snapshot', ROOT / 'scripts/handover_snapshot.py')
SNAPSHOT = importlib.util.module_from_spec(SNAPSHOT_SPEC)
SNAPSHOT_SPEC.loader.exec_module(SNAPSHOT)


class WorkspaceToolsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='mtt-tool-test-')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.env = dict(os.environ, GIT_AUTHOR_NAME='Fixture', GIT_AUTHOR_EMAIL='fixture@example.invalid',
                        GIT_COMMITTER_NAME='Fixture', GIT_COMMITTER_EMAIL='fixture@example.invalid',
                        GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL=os.devnull,
                        GIT_ALLOW_PROTOCOL='file')

    def git(self, root, *args):
        return subprocess.run(['git', '-C', str(root), *args], env=self.env, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout.strip()

    def init_repo(self, name):
        root = self.base / name
        root.mkdir()
        self.git(root, 'init', '-b', 'main')
        return root

    def commit(self, root):
        self.git(root, 'add', '.')
        self.git(root, 'commit', '-m', 'fixture', '--allow-empty')

    def run_script(self, root, name, *args):
        return subprocess.run(['bash', str(root / 'scripts' / name), *args], env=self.env,
                              cwd=self.base, text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=30)

    def test_source_check_detects_syntax_conflicts_and_broken_links(self):
        root = self.init_repo('source')
        (root / 'scripts').mkdir()
        (root / 'scripts/broken.py').write_text('def broken(:\n')
        (root / 'README.md').write_text('[missing](missing.md)\n' + '<' * 7 + ' HEAD\n')
        errors, count = CHECK.check(root)
        self.assertEqual(count, 1)
        self.assertEqual(len(errors), 3, errors)

    def test_ignored_import_is_not_hidden_by_local_file(self):
        root = self.init_repo('source')
        (root / 'scripts').mkdir()
        (root / 'scripts/main.py').write_text('import local_helper\n')
        (root / 'scripts/local_helper.py').write_text('VALUE = 1\n')
        (root / '.gitignore').write_text('scripts/local_helper.py\n')
        errors, _ = CHECK.check(root)
        self.assertTrue(any('ignored by Git' in e for e in errors))
        (root / '.gitignore').write_text('')
        self.assertEqual(CHECK.check(root)[0], [])

    def test_compose_detects_missing_optional_script(self):
        root = self.init_repo('source')
        (root / 'demos').mkdir()
        (root / 'demos/compose.yaml').write_text('command: ${WORKSPACE}/scripts/missing.py\n')
        self.assertTrue(any('absent from source' in e for e in CHECK.check(root)[0]))

    def test_missing_module_inside_existing_local_package(self):
        root = self.init_repo('source')
        (root / 'scripts/lib').mkdir(parents=True)
        (root / 'scripts/main.py').write_text('from lib.deleted import helper\n')
        self.assertTrue(any('missing local module lib.deleted' in e for e in CHECK.check(root)[0]))

    def test_subprocess_paths_include_the_entire_literal_suffix(self):
        root = self.init_repo('source')
        (root / 'scripts').mkdir()
        (root / 'demos/replay/scripts').mkdir(parents=True)
        (root / 'demos/replay/scripts/helper.py').write_text('')
        main = root / 'scripts/main.py'
        main.write_text('tool = root / "demos" / "replay" / "scripts" / "helper.py"\n')
        self.assertEqual(CHECK.check(root)[0], [])
        main.write_text('tool = root / "scripts" / "missing.py"\n')
        self.assertTrue(any('missing subprocess helper' in e for e in CHECK.check(root)[0]))

    def test_generated_and_vendor_files_are_outside_source_check(self):
        root = self.init_repo('source')
        for name in ('artifacts', 'norlab_ws', 'scripts/.venv'):
            folder = root / name
            folder.mkdir(parents=True)
            (folder / 'broken.py').write_text('not python !\n')
        (root / '.gitignore').write_text('.venv/\n')
        self.assertEqual(CHECK.check(root), ([], 0))

    def make_status_repo(self):
        root = self.init_repo('status')
        (root / 'scripts').mkdir()
        shutil.copy2(ROOT / 'scripts/status', root / 'scripts/status')
        (root / '.gitmodules').write_text('')
        (root / 'dependencies').mkdir()
        (root / 'dependencies/robot.repos').write_text('repositories: {}\n')
        (root / 'log/latest_build').mkdir(parents=True)
        (root / 'log/latest_build/events.log').write_text('')
        (root / '.gitignore').write_text('log/\n')
        self.commit(root)
        return root

    def test_dirty_development_passes_but_release_summary_fails(self):
        root = self.make_status_repo()
        (root / 'README.md').write_text('Local edit\n')
        result = self.run_script(root, 'status', '--doctor', '--allow-dirty')
        self.assertEqual(result.returncode, 0, result.stdout)
        result = self.run_script(root, 'status', '--summary')
        self.assertEqual(result.returncode, 1, result.stdout)

    def test_allow_dirty_does_not_hide_structural_error(self):
        root = self.make_status_repo()
        (root / 'broken.py').write_text('<' * 7 + ' HEAD\n')
        result = self.run_script(root, 'status', '--doctor', '--allow-dirty')
        self.assertEqual(result.returncode, 2, result.stdout)

    def make_pull_repo(self):
        child = self.init_repo('child-origin')
        (child / 'version.txt').write_text('one\n')
        self.commit(child)
        parent = self.init_repo('parent-origin')
        (parent / 'scripts').mkdir()
        shutil.copy2(ROOT / 'scripts/pull', parent / 'scripts/pull')
        self.git(parent, 'submodule', 'add', str(child), 'src/mtt_core')
        self.commit(parent)
        clone = self.base / 'clone'
        self.git(self.base, 'clone', '--recurse-submodules', str(parent), str(clone))
        return parent, child, clone

    def test_pull_detached_child_follows_parent_pin_not_branch_tip(self):
        parent, child, clone = self.make_pull_repo()
        (child / 'version.txt').write_text('two\n')
        self.commit(child)
        pinned = self.git(child, 'rev-parse', 'HEAD')
        self.git(parent / 'src/mtt_core', 'fetch')
        self.git(parent / 'src/mtt_core', 'checkout', pinned)
        self.commit(parent)
        (child / 'version.txt').write_text('three, not yet qualified\n')
        self.commit(child)
        result = self.run_script(clone, 'pull')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.git(clone / 'src/mtt_core', 'rev-parse', 'HEAD'), pinned)
        self.assertEqual(self.git(clone, 'status', '--porcelain'), '')

    def test_pull_refuses_dirty_child_before_updating_parent(self):
        parent, _, clone = self.make_pull_repo()
        previous_head = self.git(clone, 'rev-parse', 'HEAD')
        (parent / 'new.txt').write_text('remote change\n')
        self.commit(parent)
        (clone / 'src/mtt_core/version.txt').write_text('local work\n')
        result = self.run_script(clone, 'pull')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Local changes', result.stdout)
        self.assertEqual(self.git(clone, 'rev-parse', 'HEAD'), previous_head)
        self.assertEqual((clone / 'src/mtt_core/version.txt').read_text(), 'local work\n')

    def test_snapshot_preserves_current_sources_and_refuses_overwrite(self):
        _, _, clone = self.make_pull_repo()
        (clone / 'src/mtt_core/version.txt').write_text('local child edit\n')
        (clone / 'src/mtt_core/new_test.py').write_text('VALUE = 1\n')
        (clone / 'scripts/test_offline').write_text('#!/bin/bash\nexit 0\n')
        (clone / '.github/workflows').mkdir(parents=True)
        (clone / '.github/workflows/check.yml').write_text('name: check\n')
        (clone / '.env').write_text('LOCAL_TOKEN=fixture\n')
        (clone / 'data').mkdir()
        (clone / 'data/session.csv').write_text('large dataset placeholder\n')
        output = self.base / 'snapshot'
        SNAPSHOT.write_snapshot(clone, output)
        manifest = json.loads((output / 'manifest.json').read_text())
        self.assertEqual(len(manifest['repositories']), 2)
        with tarfile.open(output / 'sources.tar.gz') as archive:
            names = archive.getnames()
            self.assertIn('mtt_workspace/scripts/test_offline', names)
            self.assertIn('mtt_workspace/.github/workflows/check.yml', names)
            self.assertIn('mtt_workspace/src/mtt_core/new_test.py', names)
            self.assertNotIn('mtt_workspace/.env', names)
            self.assertNotIn('mtt_workspace/data/session.csv', names)
            self.assertEqual(archive.extractfile('mtt_workspace/src/mtt_core/version.txt').read(),
                             b'local child edit\n')
        for line in (output / 'SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ')
            self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), digest)
        with self.assertRaises(FileExistsError):
            SNAPSHOT.write_snapshot(clone, output)


if __name__ == '__main__':
    unittest.main()
