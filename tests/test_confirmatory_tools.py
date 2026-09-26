"""Exercise protocol math and qualification failures without starting ROS nodes."""

import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))
import qualify_confirmatory_session as qualification
import extract_mathis_topics as extraction
import extract_articulation_commands as articulation


def load_logic(filename):
    """Compile original definitions, excluding ROS imports and executable setup.

    This validates the real pure functions/callback state transitions, not ROS
    transport, QoS or hardware behavior. No algorithms are copied into tests.
    """
    path = ROOT / "scripts" / filename
    name = "tested_" + path.stem
    module = types.ModuleType(name)
    module.Node = object
    sys.modules[name] = module
    tree = ast.parse(path.read_text(), filename=str(path))
    allowed = set(sys.stdlib_module_names) | {"yaml"}
    tree.body = [node for node in tree.body if (
        isinstance(node, (ast.FunctionDef, ast.ClassDef, ast.Assign, ast.AnnAssign))
        or isinstance(node, ast.Import) and all(n.name.split('.')[0] in allowed for n in node.names)
        or isinstance(node, ast.ImportFrom) and node.module.split('.')[0] in allowed
    )]
    exec(compile(tree, str(path), "exec"), module.__dict__)
    return module


conductor = load_logic("mtt_experiment_conductor.py")
monitor = load_logic("mtt_confirmatory_monitor.py")


class ConfirmatoryProfileTest(unittest.TestCase):
    def test_frozen_profile_hash_and_sixty_attempts(self):
        path = ROOT / "demos/data_collection/config/session_a_confirmatory.yaml"
        manifest = json.loads(path.with_suffix(".yaml.manifest.json").read_text())
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), manifest["destination_sha256"])
        segments = conductor.load_profile(path, False)
        expected = {f"P{plan:02d}-R{rep}" for plan in range(1, 21) for rep in range(1, 4)}
        self.assertEqual({s.uid for s in segments}, expected)
        self.assertEqual(len(segments), 60)
        for segment in segments:
            self.assertGreater(segment.duration_s, 0)
            for i in range(101):
                t = segment.duration_s * i / 100
                self.assertTrue(math.isfinite(segment.phi_fn(t)))
                self.assertLessEqual(abs(segment.phi_fn(t)), math.radians(30) + 1e-12)
                self.assertGreater(segment.speed_fn(t), 0)

    def test_schedule_interpolates_declared_times(self):
        segment = conductor.build_phi_schedule({"id": "test", "waypoints": [
            {"t": 0, "phi_deg": 0}, {"t": 2, "phi_deg": 20},
            {"t": 4, "phi_deg": -20}]}, {})[0]
        for t, angle in [(-1, 0), (1, 10), (2, 20), (3, 0), (5, -20)]:
            self.assertAlmostEqual(segment.phi_fn(t), math.radians(angle))

    def test_invalid_schedules_are_rejected(self):
        for points in ([], [{"t": 0, "phi_deg": 0}],
                       [{"t": 0, "phi_deg": 0}, {"t": 0, "phi_deg": 20}],
                       [{"t": 0, "phi_deg": 0}, {"t": 1, "phi_deg": float("nan")}]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                conductor.build_phi_schedule({"id": "test", "waypoints": points}, {})

    def test_cosine_holds_and_period(self):
        cfg = dict(id="test", amplitude_deg=20, period_s=4, lead_in_s=2,
                   lead_out_s=2, evaluated_duration_s=8)
        segment = conductor.build_phi_cosine(cfg, {})[0]
        for t, angle in [(1, 0), (2, 20), (3, 0), (4, -20), (10, 0)]:
            self.assertAlmostEqual(segment.phi_fn(t), math.radians(angle))
        for key, value in [("period_s", 0), ("lead_in_s", -1), ("amplitude_deg", math.inf)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                conductor.build_phi_cosine({**cfg, key: value}, {})

    def test_duplicate_attempts_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.yaml"
            attempt = dict(uid="P01-R1", duration_s=10, speed_ms=1, phi_deg=0)
            path.write_text(yaml.safe_dump({"segments": [dict(
                id="test", type="explicit_attempts", attempts=[attempt, attempt])]}))
            with self.assertRaises(ValueError):
                conductor.load_profile(path, False)


class MonitorTest(unittest.TestCase):
    def tracker(self):
        return monitor.LiveGateTracker(monitor.ConfirmatoryGates(1, 0.5, 0.5, 0.1, 0.02))

    def test_forward_motion_passes_and_reverse_does_not_add_distance(self):
        tracker = self.tracker()
        for i in range(52):
            tracker.on_tach_sample(i * 0.02, 0.6)
        self.assertTrue(tracker.passed)
        tracker = self.tracker()
        tracker.on_tach_sample(0, 1)
        tracker.on_tach_sample(0.02, -1)
        self.assertEqual(tracker.progress()["block_distance_m"], 0)

    def test_invalid_and_discontinuous_samples_reset_progress(self):
        for t, speed in [(0.01, 1), (1, 1), (0.12, math.inf), (math.nan, 1)]:
            tracker = self.tracker()
            for i in range(6):
                tracker.on_tach_sample(i * 0.02, 1)
            tracker.on_tach_sample(t, speed)
            self.assertEqual(tracker.progress()["block_distance_m"], 0)
            self.assertFalse(tracker.passed)

    def test_stale_synthetic_and_unknown_telemetry_are_not_counted(self):
        node = object.__new__(monitor.MttConfirmatoryMonitor)
        node._now_s = lambda: 0.1
        node._maybe_emit_pass_event = lambda: None
        base = dict(telemetry_fresh=True, tachometer_is_synthetic=False, direction="Forward", speed_ms=1)
        for change in [dict(telemetry_fresh=False), dict(tachometer_is_synthetic=True),
                       dict(direction="Unknown"), dict(speed_ms=-1), dict(speed_ms=math.inf)]:
            node._tracker = self.tracker()
            node._tracker.on_tach_sample(0, 1)
            node._on_tacho(types.SimpleNamespace(**(base | change)))
            self.assertEqual(node._tracker.status(), "not_started")

    def test_transition_event_has_new_segment_flags(self):
        node = object.__new__(monitor.MttConfirmatoryMonitor)
        node._active_uid = None
        node._active_is_pause = False
        node._active_kind = "old"
        node._gates = self.tracker().gates
        node._matrix = {}
        events = []
        node._emit_event = lambda event: events.append((event, node._active_kind, node._active_is_pause))
        node._on_segment_changed("pause", dict(kind="manual_checkpoint", is_pause=True, requires_ack=True))
        self.assertEqual(events, [("pause_enter", "manual_checkpoint", True)])

    def test_dashboard_blocks_outcomes_and_ignores_nonmatrix_ids(self):
        monitor.assert_dashboard_payload_is_clean({"status": "advisory_pass", "plan_id": 1})
        with self.assertRaises(RuntimeError):
            monitor.assert_dashboard_payload_is_clean({"reference_vy": 0.1})
        for uid in ("P00-R1", "P21-R1", "P01-R4", "pause"):
            self.assertIsNone(monitor.parse_attempt_uid(uid))


class QualificationTest(unittest.TestCase):
    def verdict(self, stats):
        with patch.object(qualification, "load_bag_metadata", return_value=({"/sensor": 4}, 1000000000, 4)), \
             patch.object(qualification, "compute_topic_stats", return_value=(stats, [])):
            return qualification.qualify(Path("unused"), [dict(topic="/sensor", hard_min_hz=2, max_gap_s=0.6)])

    def test_missing_and_single_timing_samples_fail(self):
        for stats in ({}, {"/sensor": {"n": 1}}):
            self.assertFalse(self.verdict(stats)["overall_pass"])

    def test_nonmonotonic_and_nonfinite_timestamps_fail(self):
        for gaps in ([0.5, -0.1, 0.6], [0.5, 0, 0.5], [0.5, math.nan, 0.5]):
            self.assertFalse(self.verdict({"/sensor": dict(n=4, first=0, last=1, gaps=gaps)})["overall_pass"])

    def test_valid_timing_passes(self):
        self.assertTrue(self.verdict({"/sensor": dict(n=4, first=0, last=1, gaps=[0.3, 0.3, 0.4])})["overall_pass"])

    def test_empty_contract_and_invalid_limits_fail(self):
        for contract in ([], [dict(topic="/sensor", hard_min_hz=math.nan)]):
            with self.assertRaises(ValueError):
                qualification.validate_required_topics(contract)


class BagExportTest(unittest.TestCase):
    def test_help_without_ros(self):
        for path in ("extract_mathis_topics.py", "extract_articulation_commands.py", "scripts/qualify_confirmatory_session.py"):
            result = subprocess.run([sys.executable, str(ROOT / path), "--help"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_ambiguous_bag_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("a", "b"):
                (root / name).mkdir()
                (root / name / "bag_0.mcap").touch()
            with self.assertRaises(ValueError):
                extraction.find_mcap_bag_directory(root)
            self.assertEqual(extraction.find_mcap_bag_directory(root / "a" / "bag_0.mcap"), str(root / "a"))

    def test_normalized_commands_are_not_converted_to_degrees(self):
        self.assertEqual(articulation.command_degrees("/articulation_servo/steer_cmd", 0.5), "")
        self.assertAlmostEqual(articulation.command_degrees("/articulation_servo/setpoint_rad", math.pi / 2), 90)
