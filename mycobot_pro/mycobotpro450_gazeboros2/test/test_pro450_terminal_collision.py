"""Check the cable terminal against the mesh rim and the shared robot definition."""
import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET


package = Path(__file__).resolve().parents[1]


class TerminalCollisionTests(unittest.TestCase):
    def setUp(self):
        self.urdf = ET.parse(package / 'config/mycobot_pro_450_force_gripper.urdf').getroot()
        self.srdf = ET.parse(package / 'config/firefighter.srdf').getroot()
        self.link = self.urdf.find("link[@name='gripper_cable_terminal']")

    def test_terminal_is_one_collision_cylinder_without_visual(self):
        self.assertIsNotNone(self.link)
        self.assertEqual(len(self.link.findall('collision')), 1)
        self.assertEqual(self.link.findall('visual'), [])
        cylinder = self.link.find('collision/geometry/cylinder')
        self.assertAlmostEqual(float(cylinder.get('length')), .020)
        self.assertAlmostEqual(float(cylinder.get('radius')), .006624266)

    def test_cylinder_starts_at_cover_and_extends_twenty_mm_outward(self):
        xyz = [float(x) for x in self.link.find('collision/origin').get('xyz').split()]
        rpy = [float(x) for x in self.link.find('collision/origin').get('rpy').split()]
        self.assertAlmostEqual(rpy[0], -math.pi / 2)
        self.assertAlmostEqual(xyz[1] - .01, .029504446)
        self.assertAlmostEqual(xyz[1] + .01, .049504446)
        self.assertAlmostEqual(xyz[0], -.029086814)
        self.assertAlmostEqual(xyz[2], .106890604)
        # Match the existing visual transform: both are mesh-local geometry.
        joint = self.urdf.find("joint[@name='gripper_base_to_cable_terminal']")
        self.assertEqual(joint.get('type'), 'fixed')
        self.assertEqual(joint.find('parent').get('link'), 'gripper_base')
        base = self.urdf.find("link[@name='gripper_base']/visual/origin")
        self.assertEqual(joint.find('origin').get('xyz'), base.get('xyz'))
        self.assertEqual(joint.find('origin').get('rpy'), base.get('rpy'))

    def test_radius_and_center_match_actual_recess_outer_rim(self):
        path = package.parents[1] / 'mycobot_description/urdf/mycobot_pro_450/collision_exact/pro_gripper_base_exact.stl'
        raw = path.read_bytes()
        cylinder = self.link.find('collision/geometry/cylinder')
        radius = float(cylinder.get('radius'))
        points = set()
        for record in struct.iter_unpack('<12fH', raw[84:]):
            for i in (3, 6, 9):
                x, y, z = record[i:i+3]
                radial = math.hypot(x + .029086814, z - .106890604)
                if abs(y - .029504446) < 2e-8 and abs(radial - radius) < 2e-6:
                    points.add((x, y, z))
        self.assertGreaterEqual(len(points), 40)
        self.assertLess(max(abs(math.hypot(x+.029086814, z-.106890604)-radius)
                            for x, _, z in points), 5e-8)

    def test_only_mounting_parent_is_exempt_in_existing_matrix(self):
        pairs = [set((p.get('link1'), p.get('link2'))) for p in self.srdf.findall('disable_collisions')
                 if 'gripper_cable_terminal' in (p.get('link1'), p.get('link2'))]
        self.assertEqual(pairs, [{'gripper_base', 'gripper_cable_terminal'}])
        group = self.srdf.find("group[@name='gripper']")
        self.assertIsNotNone(group.find("joint[@name='gripper_base_to_cable_terminal']"))


if __name__ == '__main__':
    unittest.main()
