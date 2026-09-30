# ROS 2 Line Follower Robot - Makefile
# ROS 2 Jazzy / Ubuntu 24.04
#
# NOTE: the ROS 2 package lives under src/line_follower_az/. Run the ROS2
# targets (build/sim/run/monitor) from the workspace root (where this file is).
# The headless targets (test/sim) run from src/line_follower_az/ and need only
# Python 3 + numpy + matplotlib — no ROS2.

WORKSPACE_DIR := $(shell pwd)
PKG_DIR := $(WORKSPACE_DIR)/src/line_follower_az
ROS_SETUP := /opt/ros/jazzy/setup.bash
WS_SETUP := $(WORKSPACE_DIR)/install/setup.bash

.PHONY: all build clean sim run monitor test sim-headless help

# Default target
all: build

# Build
build:
	@echo "Building ROS 2 workspace..."
	colcon build --symlink-install

# Clean
clean:
	@echo "Cleaning workspace..."
	rm -rf build/ install/ log/

# Simulation
# Launches Simulation, Nodes, and Telemetry in a single split Tilix window.
sim: clean build
	@echo "Launching simulation..."
	tilix --action=app-new-session -e bash -c "source $(ROS_SETUP) && source $(WS_SETUP) && ros2 launch line_follower_az simulation.launch.py; exec bash" \
	--action=session-add-right -e bash -c "source $(ROS_SETUP) && source $(WS_SETUP) && sleep 3 && ros2 launch line_follower_az nodes.launch.py; exec bash" \
	--action=session-add-down -e bash -c "source $(ROS_SETUP) && source $(WS_SETUP) && ros2 topic echo /cmd_vel; exec bash"

# Run Nodes Only
# Alternative: Run only the ROS 2 nodes when simulation is already running elsewhere.
run: build
	@echo "Launching line follower nodes..."
	bash -c "source $(ROS_SETUP) && source $(WS_SETUP) && ros2 launch line_follower_az nodes.launch.py"

# Monitoring / PID Tuning
# Launch rqt for live PID parameter tuning and ROS 2 node graph visualization.
monitor:
	@echo "Launching rqt..."
	tilix \
		--action=app-new-session \
		-e "bash -c 'source $(ROS_SETUP) && source $(WS_SETUP) && rqt; exec bash'"

# Headless 2D simulator (no ROS2 needed -- runs on any OS)
sim-headless:
	@echo "Running headless 2D simulator (3 laps)..."
	cd $(PKG_DIR) && python3 -m line_follower_az.simulator --laps 3

# Headless tests (no ROS2 / Gazebo needed -- runs on any OS)
test:
	@echo "Running headless verification (sensor math, PID sign chain, simulator)..."
	cd $(PKG_DIR) && python3 scripts/test_sensor_math.py
	cd $(PKG_DIR) && python3 scripts/test_pid_sign_chain.py
	cd $(PKG_DIR) && python3 scripts/test_simulator.py

# Help
help:
	@echo ""
	@echo "ROS 2 Line Follower Robot"
	@echo "========================="
	@echo ""
	@echo "Available commands:"
	@echo ""
	@echo "  make build"
	@echo "      Build the ROS 2 workspace using colcon."
	@echo ""
	@echo "  make clean"
	@echo "      Remove build, install, and log directories."
	@echo ""
	@echo "  make sim"
	@echo "      Build and launch the Gazebo simulation, ROS 2 nodes,"
	@echo "      and /cmd_vel telemetry in a split Tilix session."
	@echo ""
	@echo "  make run"
	@echo "      Build and launch the line-following nodes only."
	@echo "      Use this when the simulation is already running."
	@echo ""
	@echo "  make monitor"
	@echo "      Launch rqt for live parameter tuning and ROS graph"
	@echo "      visualization."
	@echo ""
	@echo "  make sim-headless"
	@echo "      Run the headless 2D simulator (no ROS2 needed)."
	@echo ""
	@echo "  make test"
	@echo "      Run the headless verification suite (no ROS2 needed)."
	@echo ""
	@echo "  make help"
	@echo "      Display this help message."
	@echo ""
	@echo "Environment:"
	@echo "  ROS 2      : Jazzy"
	@echo "  Workspace  : $(WORKSPACE_DIR)"
	@echo ""