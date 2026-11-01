"""robot_dog constants shared by the Isaac Sim scripts (plain Python, no omni imports)."""
import os

STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}   # rad
EFFORT, VELOCITY = 25.0, 30.0                      # N*m, rad/s (placeholders, as in the URDF)
SPAWN_Z = 0.32
# sensor poses in base_link (m), identical to the URDF frames
IMU_XYZ = (0.0, 0.0, 0.048)
LASER_XYZ = (0.08, 0.0, 0.085)
CAMERA_XYZ = (0.178, 0.0, 0.0575)
CAMERA_HFOV = 1.204  # rad, same as Gazebo
CAMERA_SIZE = (640, 480)

DEFAULT_URDF = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'urdf', 'robot_dog_rl.urdf'))
