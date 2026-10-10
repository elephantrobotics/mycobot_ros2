"""Current-state adapter precedes, rather than replaces, collision adapters."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

launch = Path(__file__).resolve().parents[1] / 'launch' / 'slider.launch.py'


class PlanningStartConfigTests(unittest.TestCase):
    def test_all_pipelines_keep_existing_adapters_and_goal_configuration(self):
        tree = ast.parse(launch.read_text(encoding='utf-8'))
        func = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                    and n.name == 'use_current_planning_start')
        ns = {}
        exec(compile(ast.Module(body=[func], type_ignores=[]), str(launch), 'exec'), ns)
        config = SimpleNamespace(planning_pipelines={
            'planning_pipelines': ['ompl', 'chomp', 'pilz'],
            'default_planning_pipeline': 'ompl',
            'ompl': {'request_adapters': 'TimeParameterization FixStartStateCollision', 'planner_configs': {'RRT': {}}},
            'chomp': {'request_adapters': 'FixStartStateBounds'},
            'pilz': {'request_adapters': '', 'default_planner_config': 'PTP'},
        })
        apply = ns['use_current_planning_start']
        self.assertIs(apply(config), config)
        apply(config)  # No duplicate adapter after repeated stack construction.
        prefix = 'pro450_gazebo/UseCurrentStartState'
        self.assertEqual(config.planning_pipelines['ompl']['request_adapters'],
                         prefix + ' TimeParameterization FixStartStateCollision')
        self.assertEqual(config.planning_pipelines['chomp']['request_adapters'], prefix + ' FixStartStateBounds')
        self.assertEqual(config.planning_pipelines['pilz']['request_adapters'], prefix)
        self.assertEqual(config.planning_pipelines['ompl']['planner_configs'], {'RRT': {}})
        self.assertEqual(config.planning_pipelines['pilz']['default_planner_config'], 'PTP')


if __name__ == '__main__':
    unittest.main()
