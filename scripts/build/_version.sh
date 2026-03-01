#!/bin/bash
# Centralized STM32CubeIDE version configuration.
#
# Docker tag mapping (see https://github.com/xanderhendriks/action-build-stm32cubeide#stm32-cube-ide-versions):
#   Docker tag = STM32CubeIDE minor version - 3
#   e.g. STM32CubeIDE 1.10.1 → 7.0, STM32CubeIDE 1.18.0 → 15.0
#
# Override by setting DOCKER_STM32CUBEIDE_VERSION before sourcing this file
# or before calling any docker_*.sh script.

DOCKER_STM32CUBEIDE_VERSION="${DOCKER_STM32CUBEIDE_VERSION:-15.0}"

# STM32CubeIDE >= 1.18.0 (docker tag >= 15.0) replaced -import with -importAll
_major="${DOCKER_STM32CUBEIDE_VERSION%%.*}"
if [ "${_major}" -ge 15 ] 2>/dev/null; then
	export STM32_IMPORT_FLAG="-importAll"
else
	export STM32_IMPORT_FLAG="-import"
fi
unset _major
