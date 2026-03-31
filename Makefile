# ==============================================================================
# Go2 CycloneDDS ROS2 SDK — Makefile
#
# Usage:
#   make build     — Build the Docker image
#   make up        — Start the container (detached)
#   make down      — Stop the container
#   make logs      — Tail container logs
#   make shell     — Open a bash shell in the running container
#   make topics    — List available ROS2 topics
#   make clean     — Remove Docker image and build artifacts
#   make rebuild   — Force rebuild (no cache)
#   make safe           — Start in read-only mode (robot won't move)
#   make setup-network  — Add static IP for Go2 communication
#   make check-network  — Check network status and Go2 connectivity
# ==============================================================================

COMPOSE := docker compose -f docker/compose.yml
CONTAINER := go2_cyclonedds

.PHONY: build up down logs shell topics clean rebuild safe setup-network check-network

build:
	$(COMPOSE) build

rebuild:
	$(COMPOSE) build --no-cache

up:
	xhost +local:root
	$(COMPOSE) up -d
	@echo "Container started. Run 'make logs' to view output."

safe:
	xhost +local:root
	ENABLE_CMD_VEL=false $(COMPOSE) up -d
	@echo "Container started in READ-ONLY mode. Robot won't move."

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f --tail=100

shell:
	docker exec -it $(CONTAINER) bash

topics:
	docker exec -it $(CONTAINER) bash -c \
		"source /ros2_ws/install/setup.bash && ros2 topic list"

setup-network:
	sudo bash scripts/setup_network.sh

check-network:
	sudo bash scripts/setup_network.sh --check-only

clean:
	$(COMPOSE) down --rmi all --volumes --remove-orphans 2>/dev/null || true
	@echo "Cleaned."
