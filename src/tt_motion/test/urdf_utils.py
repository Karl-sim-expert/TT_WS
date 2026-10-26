"""Minimal stand-in for xacro, enough for tt_robot.urdf.xacro, so the kinematics tests run
without ROS: resolves $(arg ...) with the declared defaults, evaluates ${...} arithmetic, inlines
panda_arm.urdf and drops the ros2_control / gazebo blocks."""
import os
import re
import xml.etree.ElementTree as ET

DESC = os.path.join(os.path.dirname(__file__), '..', '..', 'tt_description', 'urdf')


def processed_urdf():
    with open(os.path.join(DESC, 'tt_robot.urdf.xacro')) as f:
        text = f.read()
    text = re.sub(r'<ros2_control.*?</ros2_control>', '', text, flags=re.S)
    text = re.sub(r'<gazebo>.*?</gazebo>', '', text, flags=re.S)
    text = re.sub(r'<gazebo reference.*?</gazebo>', '', text, flags=re.S)
    args = dict(re.findall(r'<xacro:arg name="(\w+)" default="([^"]*)"', text))
    text = re.sub(r'\$\(arg (\w+)\)', lambda m: args[m.group(1)], text)
    text = re.sub(r'\$\{([^}]*)\}', lambda m: str(eval(m.group(1))), text)  # simple arithmetic only
    root = ET.fromstring(text)
    out = ET.Element('robot', name='tt_robot')
    panda = ET.parse(os.path.join(DESC, 'panda_arm.urdf')).getroot()
    for el in list(panda):
        out.append(el)
    for el in root:
        if el.tag in ('link', 'joint'):
            out.append(el)
    return ET.tostring(out, encoding='unicode')
