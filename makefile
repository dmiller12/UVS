
OS := $(shell uname)

ifeq ($(OS), Linux)
    DOCKER_DISPLAY = --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--env="DISPLAY" 
else ifeq ($(OS), Darwin)
    DOCKER_DISPLAY = --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--env DISPLAY=host.docker.internal:0
endif

DOCKER_VOLUMES = \
	--volume="$(shell pwd)/uvs_bridge":"/root/catkin_ws/src/uvs_bridge"
DOCKER_ENV_VARS = \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1"

DOCKER_ARGS = ${DOCKER_VOLUMES} ${DOCKER_ENV_VARS} ${DOCKER_DISPLAY}

.PHONY: build
build:
	@docker build -t uvs .

.PHONY: term
term:
	docker run -it --net=host --user 1001 --privileged ${DOCKER_ARGS} uvs bin/bash
