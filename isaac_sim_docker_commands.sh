ISAAC_SIM_DIR="../../IsaacLab" # Docker environment for IsaacSim doesn't work if the directory isn't in home
export ISAAC_SCRIPTS_DIR=$(pwd)/Isaac-lab-scripts
# $ISAAC_SIM_DIR/docker/container.py build
$ISAAC_SIM_DIR/docker/container.py start --files $ISAAC_SCRIPTS_DIR/docker-compose.custom.yaml
$ISAAC_SIM_DIR/docker/container.py enter

# ./docker/container.py stop # To stop the container
