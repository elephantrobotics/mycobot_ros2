"""Prevent competing gripper writers and incorrect native mimic initialization."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


package = Path(__file__).resolve().parents[1]


class GazeboMimicTests(unittest.TestCase):
    def setUp(self):
        self.urdf = ET.parse(package / 'config/mycobot_pro_450_force_gripper.urdf').getroot()
        self.control = ET.parse(package / 'config/firefighter.ros2_control.xacro').getroot().find('.//ros2_control')
        self.followers = {joint.get('name'): joint.find('mimic') for joint in self.urdf.findall('joint')
                          if joint.find('mimic') is not None}

    def test_all_five_followers_use_same_master_and_sign_as_urdf(self):
        native = {joint.get('name'): joint for joint in self.control.findall('joint')
                  if joint.find("param[@name='mimic']") is not None}
        self.assertEqual(len(self.followers), 5)
        self.assertEqual(set(native), set(self.followers))
        for name, mimic in self.followers.items():
            with self.subTest(joint=name):
                self.assertEqual(native[name].find("param[@name='mimic']").text, mimic.get('joint'))
                self.assertEqual(float(native[name].find("param[@name='multiplier']").text),
                                 float(mimic.get('multiplier')))
                self.assertEqual(float(mimic.get('offset')), 0.0)
                self.assertIsNotNone(native[name].find("command_interface[@name='position']"))

    def test_nonzero_spawn_initializes_followers_with_correct_signed_opening(self):
        for opening in (0.0, .45, 1.0):
            for name, mimic in self.followers.items():
                expression = self.control.find(f"joint[@name='{name}']/state_interface[@name='position']/param").text
                value = eval(expression[2:-1], {'__builtins__': {}},
                             {'initial_positions': {'gripper_controller': opening}})
                self.assertAlmostEqual(value, opening * float(mimic.get('multiplier')))
                limit = self.urdf.find(f"joint[@name='{name}']/limit")
                self.assertGreaterEqual(value, float(limit.get('lower')))
                self.assertLessEqual(value, float(limit.get('upper')))

    def test_external_mimic_plugins_cannot_override_friction_or_position(self):
        self.assertFalse(any('mimic_joint_plugin' in plugin.get('filename', '')
                             for plugin in self.urdf.findall('gazebo/plugin')))
        for name in self.followers:
            self.assertEqual(float(self.urdf.find(f"joint[@name='{name}']/dynamics").get('friction')), .1)
        limit = self.urdf.find("joint[@name='gripper_controller']/limit")
        self.assertEqual((float(limit.get('lower')), float(limit.get('upper'))), (0.0, 1.0))

    def test_published_joint_names_and_controller_claims_stay_independent(self):
        config = yaml.safe_load((package / 'config/ros2_controllers.yaml').read_text())
        expected = [f'joint{i}' for i in range(1, 7)] + ['gripper_controller']
        self.assertEqual(config['joint_state_broadcaster']['ros__parameters']['joints'], expected)
        self.assertEqual(config['arm_controller']['ros__parameters']['joints'], expected[:6])
        self.assertEqual(config['pro_gripper_controller']['ros__parameters']['joints'], expected[6:])


if __name__ == '__main__':
    unittest.main()
