#!/usr/bin/env bash
# Create (or update) the Isaac Lab 3.0 project "robot_dog_lab" and install the robot_dog task.
#
#   ./setup_robot_dog_lab.sh [ISAACLAB_DIR] [PROJECTS_DIR]
#     ISAACLAB_DIR  Isaac Lab source checkout   (default: ~/IsaacLab)
#     PROJECTS_DIR  where the project is created (default: ~/projects)
#
# Running it again only refreshes the robot_dog files in the existing project.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACLAB_DIR="${1:-$HOME/IsaacLab}"
PROJECTS_DIR="${2:-$HOME/projects}"
PROJECT="$PROJECTS_DIR/robot_dog_lab"
TASK="RobotDog-Velocity-Flat"

command -v uv >/dev/null || { echo "uv not found: install it first (https://docs.astral.sh/uv/)"; exit 1; }
[ -d "$ISAACLAB_DIR" ] || { echo "Isaac Lab checkout not found at $ISAACLAB_DIR (pass it as the first argument)"; exit 1; }

if [ ! -d "$PROJECT/src/robot_dog_lab" ]; then
    echo ">> generating Isaac Lab project at $PROJECT"
    mkdir -p "$PROJECTS_DIR"
    # default template content brings the rsl_rl dependency; its cartpole example task can stay
    (cd "$ISAACLAB_DIR" && uv run isaaclab --new --non_interactive \
        --project_path "$PROJECTS_DIR" --name robot_dog_lab --author "${USER:-robot_dog}")
fi

echo ">> copying robot_dog asset, URDF/meshes and task into $PROJECT/src/robot_dog_lab"
cp -r "$HERE/robot_dog_lab/." "$PROJECT/src/robot_dog_lab/"

cd "$PROJECT"
echo ">> checking the task registration"
if uv run --extra isaacsim isaaclab list_envs | grep -q "$TASK"; then
    echo ">> $TASK is registered"
else
    echo ">> $TASK not found in 'isaaclab list_envs' output, check the messages above"
    exit 1
fi

cat <<EOF

Done. From $PROJECT:
  quick test : uv run --extra isaacsim isaaclab train --rl_library rsl_rl --task $TASK --num_envs 64 --max_iterations 5
  train      : uv run --extra isaacsim isaaclab train --rl_library rsl_rl --task $TASK
  play       : uv run --extra isaacsim isaaclab play --rl_library rsl_rl --task $TASK --checkpoint latest --viz kit
EOF
