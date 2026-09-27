"""Candidate generation must preserve extrinsics and the original calibration."""

import math
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from lib.calibration_xacro import REQUIRED_PROPERTIES, calibration_properties, write_candidate


class CalibrationXacroTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='mtt-calib-test-')
        self.addCleanup(self.temp.cleanup)
        self.template = Path(self.temp.name) / 'template.xacro'
        self.output = Path(self.temp.name) / 'candidate.xacro'
        properties = ''.join(
            f'<xacro:property name="{name}" value="0 0 {math.pi / 2}"/>'
            for name in sorted(REQUIRED_PROPERTIES))
        self.template.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
                                 + properties + '</robot>')

    def test_preserves_yaw_and_other_sensor_mounts(self):
        original = self.template.read_bytes()
        _, before = calibration_properties(self.template)
        write_candidate(self.template, self.output, (1, 2, 3), 0.01, -0.02)
        _, after = calibration_properties(self.output)
        self.assertEqual(self.template.read_bytes(), original)
        self.assertEqual(set(before), set(after))
        for name in REQUIRED_PROPERTIES - {'hesai_calib_xyz', 'hesai_calib_rpy'}:
            self.assertEqual(before[name].attrib, after[name].attrib)
        self.assertAlmostEqual(float(after['hesai_calib_rpy'].attrib['value'].split()[2]), math.pi / 2)

    def test_refuses_overwrite(self):
        self.output.write_text('previous measured calibration')
        with self.assertRaises(FileExistsError):
            write_candidate(self.template, self.output, (1, 2, 3), 0, 0)
        self.assertEqual(self.output.read_text(), 'previous measured calibration')

    def test_rejects_nonfinite_or_incomplete_candidate(self):
        with self.assertRaises(ValueError):
            write_candidate(self.template, self.output, (math.nan, 2, 3), 0, 0)
        self.assertFalse(self.output.exists())
        self.template.write_text('<robot/>')
        with self.assertRaises(ValueError):
            write_candidate(self.template, self.output, (1, 2, 3), 0, 0)
