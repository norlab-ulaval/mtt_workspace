"""Protect the recovered research dependency and its public entrypoints."""

import ast
import csv
import importlib
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from lib import mtt_motion_research as research  # noqa: E402


class ResearchEntrypointsTest(unittest.TestCase):
    def test_tacho_provenance_requires_explicit_evidence(self):
        cases = [({}, 0), ({'has_real_tacho': False}, 0),
                 ({'has_real_tacho': True}, 1), ({'tach_is_synthetic': False}, 1),
                 ({'has_real_tacho': False, 'tach_is_synthetic': False}, 0),
                 ({'has_real_tacho': True, 'tach_is_synthetic': True}, 0)]
        for metadata, expected in cases:
            with self.subTest(metadata=metadata):
                row = {'t': 0, 'icp_x': 0, 'icp_y': 0, 'icp_yaw': 0, **metadata}
                derived = research.derive_research_rows([row], session_name='synthetic')
                self.assertEqual(derived[0]['has_real_tacho_research'], expected)

    def test_nonpositive_time_intervals_are_not_valid_research_samples(self):
        for second_time in (0.0, -0.1):
            with self.subTest(second_time=second_time):
                rows = research.derive_research_rows([
                    {'t': 0, 'icp_x': 0, 'icp_y': 0, 'icp_yaw': 0, 'icp_quality_ok': True},
                    {'t': second_time, 'icp_x': 0.01, 'icp_y': 0, 'icp_yaw': 0,
                     'icp_quality_ok': True},
                ], session_name='synthetic')
                self.assertEqual(rows[0]['research_quality_ok'], 0)
                self.assertEqual(rows[1]['research_quality_ok'], 0)
                self.assertIn('invalid_dt', rows[1]['quality_reason'])

    def test_zero_imu_yaw_rate_is_valid_data(self):
        rows = research.derive_research_rows([
            {'t': 0, 'icp_x': 0, 'icp_y': 0, 'icp_yaw': 0, 'imu_angular_velocity_z': 0},
        ], session_name='synthetic')
        self.assertEqual(rows[0]['imu_yaw_rate_rad_s'], 0.0)

    def test_forward_speed_does_not_reverse_at_pi_wrap(self):
        rows = research.derive_research_rows([
            {'t': 0, 'icp_x': 0, 'icp_y': 0, 'icp_yaw': math.pi - 0.01,
             'odom_x': 0, 'odom_y': 0, 'odom_yaw': math.pi - 0.01},
            {'t': 0.1, 'icp_x': -0.1, 'icp_y': 0, 'icp_yaw': -math.pi + 0.01,
             'odom_x': -0.1, 'odom_y': 0, 'odom_yaw': -math.pi + 0.01},
        ], session_name='synthetic')
        self.assertAlmostEqual(rows[1]['icp_speed_ms'], 1.0)
        self.assertAlmostEqual(rows[1]['odom_speed_ms'], 1.0)
        self.assertAlmostEqual(rows[1]['icp_yaw_rate_rad_s_derived'], 0.2)

    def test_legacy_heading_alias_survives_multiple_rows(self):
        rows = research.derive_research_rows([
            {'t': 0, 'icp_x': 0, 'icp_y': 0, 'icp_heading': 0},
            {'t': 0.1, 'icp_x': 0.1, 'icp_y': 0, 'icp_heading': 0},
        ], session_name='synthetic')
        self.assertAlmostEqual(rows[1]['icp_speed_ms'], 1.0)

    def test_build_dataset_cli_preserves_input_and_exports_rows(self):
        with tempfile.TemporaryDirectory(prefix='mtt-research-test-') as directory:
            session = Path(directory) / 'synthetic'
            source = session / 'postprocess_dataset/motion_model_dataset.csv'
            source.parent.mkdir(parents=True)
            with source.open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=['t', 'icp_x', 'icp_y', 'icp_yaw'])
                writer.writeheader()
                for index in range(30):
                    writer.writerow({'t': index * 0.1, 'icp_x': index * 0.02, 'icp_y': 0, 'icp_yaw': 0})
            before = source.read_bytes()
            result = subprocess.run([
                sys.executable, str(SCRIPTS / 'build_motion_research_dataset.py'), str(session)],
                cwd='/tmp', text=True, capture_output=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            self.assertEqual(source.read_bytes(), before)
            with (session / 'motion_research/dataset.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 30)
            self.assertEqual(rows[1]['session'], 'synthetic')

    def test_all_imported_research_symbols_exist(self):
        users = []
        for path in SCRIPTS.glob('*.py'):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.ImportFrom) and node.module == 'lib.mtt_motion_research':
                    users.append(path)
                    for symbol in node.names:
                        self.assertTrue(hasattr(research, symbol.name), f'{path.name}: {symbol.name}')
        self.assertGreaterEqual(len(set(users)), 6)

    def test_entrypoints_help_without_data_or_ros(self):
        names = [
            'build_motion_research_dataset.py', 'diagnose_motion_model_failure.py',
            'evaluate_command_model_progression.py', 'evaluate_motion_model_suite.py',
            'kfold_motion_model_tuning.py', 'plot_motion_research_report.py',
        ]
        for name in names:
            with self.subTest(script=name):
                result = subprocess.run([sys.executable, str(SCRIPTS / name), '--help'],
                                        cwd='/tmp', capture_output=True, text=True, timeout=20,
                                        env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('usage:', result.stdout)

    def test_finite_data_and_coordinate_conventions(self):
        for value in (None, '', 'nan', 'inf', '-inf'):
            self.assertIsNone(research.parse_float(value))
        self.assertEqual(research.finite_stats([math.nan, math.inf])['count'], 0)
        self.assertEqual(research.nominal_curvature(0.0), 0.0)
        self.assertAlmostEqual(research.nominal_curvature(0.2), -research.nominal_curvature(-0.2))
        # A world +Y error is forward when the predicted body faces +Y.
        x, y, yaw = research.se2_log_error(0, 0, math.pi / 2, 0, 1, math.pi / 2)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)


if __name__ == '__main__':
    unittest.main()
