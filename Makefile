# Day-to-day commands, all runnable from the repo root. `make` lists them.
#
# The sim is Unity (unity/SparSim, MuJoCo plugin); the container runs
# nothing by default, see docker/entrypoint.sh. `make ros2_container` only
# builds the image and starts the container; it does not build the code,
# even on a fresh clone. Building and running the code are both on you:
#
#   make ros2_container    # build the image, start the container, drop you into a shell
#   (in that shell) colcon build --symlink-install   # first build
#   (in that shell) cd build/spar_ground && make   # NOT cmake .. && make;
#                    colcon's build dir caches its own source path, plain make is correct
#   (in that shell) ros2 launch spar_bringup autonomy.launch.py world:=blank
#                    logs go to logs/runNNN automatically (ROS_LOG_DIR is set
#                    once per container start, see docker/entrypoint.sh)
#
# Ctrl-C the launch and rerun it after every rebuild. `make shell` gets you
# back into a running container later without restarting anything. The one
# fully automated exception is `make smoke`, which brings the whole stack
# up itself for an unattended end-to-end check.
#
# If you restart Unity, restart the containers after it: Unity owns /clock, and
# a fresh sim rewinds time, which clears TF buffers, puts Nav2's lifecycle
# servers to sleep ("Action server is inactive"), and rewinds the clock under
# PX4, whose lockstep only moves forward. Same order for the air track.
COMPOSE   := docker compose -f docker/compose.yaml
CONTAINER := spar
LAUNCH    := spar_bringup autonomy.launch.py
WORLD     ?= blank
VENV      := .venv
ROS_ENV   := source /ws/scripts/env.sh

# The *_air targets below are two-line aliases onto this TRACK switch,
# which points the same recipes at the air container (PX4 + spar_air).
# The compose profile means only the air track ever builds the PX4 image;
# a bare `docker compose up` can't start it by accident. shut_down/clean
# always include the profile so they act on the whole project.
TRACK ?= ground
WORKDIRS := ground/build ground/install logs
SMOKE    := /ws/scripts/smoke_test.sh
ifeq ($(TRACK),air)
  COMPOSE   := $(COMPOSE) --profile air
  CONTAINER := spar-air
  LAUNCH    := spar_air air.launch.py
  WORKDIRS  += air/src air/build air/install logs/air
  SMOKE     := /ws/scripts/smoke_test_air.sh
endif
COMPOSE_ALL := docker compose -f docker/compose.yaml --profile air

# The editor version the project itself records, so the two can't drift.
UNITY_VERSION := $(shell sed -n 's/^m_EditorVersion: //p' unity/SparSim/ProjectSettings/ProjectVersion.txt)
UNITY_APP    ?= /Applications/Unity/Hub/Editor/$(UNITY_VERSION)/Unity.app
UNITY_EDITOR ?= $(UNITY_APP)/Contents/MacOS/Unity

.PHONY: help ros2_container ros2_container_air clean shut_down unity-headless unity-gui unity-stop shell shell_air tail tail_air smoke smoke_air map rviz

help:           ## list commands
	@grep -E '^[a-z0-9_-]+: .*##' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  make %-10s %s\n", $$1, $$2}'

ros2_container: ## build + start the container (nothing built or launched yet) and drop you into a shell
	@mkdir -p $(WORKDIRS)
	WORLD=$(WORLD) $(COMPOSE) up --build -d
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && exec bash'

ros2_container_air: ## same, for the air track (first build compiles PX4, takes a while)
	$(MAKE) ros2_container TRACK=air

shut_down:      ## stop and remove the containers
	$(COMPOSE_ALL) down

clean: shut_down ## shut down, then remove both tracks' build trees (forces a full rebuild on the next `make ros2_container`)
	rm -rf ground/build ground/install air/build air/install

unity-headless: ## run the sim headless (no editor window; make unity-stop ends it)
	@mkdir -p logs
	@nohup "$(UNITY_EDITOR)" -batchmode -projectPath $(CURDIR)/unity/SparSim \
	  -executeMethod SparBootstrap.Play -logFile - \
	  > logs/unity-headless.log 2>&1 & echo "headless Unity starting (logs/unity-headless.log)"

unity-gui:      ## open the Unity editor and press Play for you (camera perception)
	@pkill -f "Unity.*-batchmode.*SparSim" 2>/dev/null; true
	open -na "$(UNITY_APP)" --args -projectPath $(CURDIR)/unity/SparSim \
	  -executeMethod SparBootstrap.Play

unity-stop:     ## stop the headless sim (the GUI editor you close yourself)
	@pkill -f "Unity.*-batchmode.*SparSim" && echo "stopped" || echo "not running"

shell:          ## a shell inside the container, ROS already sourced
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && exec bash'

shell_air:      ## a shell inside the air container
	$(MAKE) shell TRACK=air

tail:           ## echo /rosout live, every node's log messages merged (fails if the track's launch isn't running)
	@docker exec $(CONTAINER) pgrep -f 'ros2 launch $(LAUNCH)' >/dev/null \
	  || { echo "$(LAUNCH) isn't running"; exit 1; }
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && ros2 topic echo /rosout'

tail_air:       ## echo the air track's /rosout live
	$(MAKE) tail TRACK=air

rviz:           ## rviz2 in a browser (macOS can't pop an X window out of the container); Ctrl-C to stop
	@open "http://localhost:6080/vnc.html?autoconnect=true&resize=remote"
	docker exec -it $(CONTAINER) /ws/scripts/rviz.sh

smoke:          ## end-to-end test of the whole behavior arc (~4 min, ends in PASS)
	docker exec $(CONTAINER) bash -lc '$(ROS_ENV) && $(SMOKE)'

smoke_air:      ## end-to-end test of the air track (~4-6 min, ends in PASS)
	$(MAKE) smoke TRACK=air

map: $(VENV)    ## regenerate the AMCL map from the Unity scene (dump MJCF -> lint -> rasterize)
	@mkdir -p logs
	@pkill -f "Unity.*-batchmode.*SparSim" 2>/dev/null; true
	SPAR_DUMP_MJCF=$(CURDIR)/logs/$(WORLD).xml \
	  "$(UNITY_EDITOR)" -batchmode -projectPath $(CURDIR)/unity/SparSim \
	  -executeMethod SparBootstrap.DumpWorld -logFile - \
	  > logs/unity-dump.log 2>&1 || { tail -20 logs/unity-dump.log; exit 1; }
	$(VENV)/bin/python scripts/lint_world.py logs/$(WORLD).xml \
	  ground/src/spar_bringup/config/autonomy_$(WORLD).yaml
	$(VENV)/bin/python scripts/rasterize_map.py logs/$(WORLD).xml ground/src/spar_bringup/maps

$(VENV):
	python3 -m venv $(VENV) && $(VENV)/bin/pip install mujoco pyyaml
