"""Write a reviewable calibration candidate without erasing other sensor mounts."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


NAMESPACE = 'http://www.ros.org/wiki/xacro'
REQUIRED_PROPERTIES = {
    f'{sensor}_calib_{component}'
    for sensor in ('hesai', 'imu', 'mti10_right', 'mti10_left', 'zed2i', 'rsairy')
    for component in ('xyz', 'rpy')
}


def calibration_properties(path):
    tree = ET.parse(path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    properties = {node.attrib['name']: node for node in tree.iter(f'{{{NAMESPACE}}}property')}
    missing = REQUIRED_PROPERTIES - properties.keys()
    if missing:
        raise ValueError(f'Incomplete calibration template: {sorted(missing)}')
    for name in REQUIRED_PROPERTIES:
        values = properties[name].attrib['value'].split()
        if len(values) != 3 or not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f'{name} must contain three finite numbers')
    return tree, properties


def write_candidate(template, output, xyz, roll, pitch):
    """Preserve template yaw and all other mounts; refuse to overwrite any file."""
    tree, properties = calibration_properties(template)
    xyz = tuple(float(value) for value in xyz)
    if len(xyz) != 3 or not all(math.isfinite(value) for value in (*xyz, roll, pitch)):
        raise ValueError('Candidate position and tilt must be finite')
    yaw = float(properties['hesai_calib_rpy'].attrib['value'].split()[2])
    properties['hesai_calib_xyz'].set('value', ' '.join(f'{value:.9f}' for value in xyz))
    properties['hesai_calib_rpy'].set('value', f'{roll:.9f} {pitch:.9f} {yaw:.16g}')
    tree.getroot().insert(0, ET.Comment(
        ' UNVALIDATED CANDIDATE. Only Hesai position/tilt updated; yaw and other '
        'mounts preserved. Historical comments below describe the template. '
        'Review frame convention and measured displacement before deployment. '))
    ET.register_namespace('xacro', NAMESPACE)
    with Path(output).open('xb') as stream:
        tree.write(stream, encoding='utf-8', xml_declaration=True)
