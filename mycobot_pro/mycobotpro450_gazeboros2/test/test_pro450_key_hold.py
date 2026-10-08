"""Key-release decision does not depend on the Linux repeat rate."""
import ast
from pathlib import Path
import unittest

source = (Path(__file__).resolve().parents[1] / "scripts" / "teleop_keyboard_gazebo.py").read_text(encoding="utf-8")
tree = ast.parse(source)
function = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "hold_release_action")
namespace = {}
exec(compile(ast.Module(body=[function], type_ignores=[]), "hold_release_action", "exec"), namespace)
hold_release_action = namespace["hold_release_action"]


class KeyHoldTests(unittest.TestCase):
    def test_held_key_ignores_autorepeat_release(self):
        self.assertEqual(hold_release_action(True), "ignore")

    def test_lifted_key_releases(self):
        self.assertEqual(hold_release_action(False), "release")

    def test_unknown_state_only_debounces(self):
        self.assertEqual(hold_release_action(None), "debounce")


if __name__ == "__main__":
    unittest.main()
