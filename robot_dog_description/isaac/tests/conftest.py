import importlib.util
import os
import sys

# make robot_dog_sim importable without installing it
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

# these need torch + isaaclab: run them in the Isaac Lab uv env, skip them under the system python
if importlib.util.find_spec('torch') is None:
    collect_ignore = ['test_actuator.py', 'test_observations.py']
