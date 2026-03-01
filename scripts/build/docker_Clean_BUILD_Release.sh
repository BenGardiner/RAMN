#!/bin/bash
# Clean-build ECU(s) with the STM32CubeIDE Docker container.
# Usage: docker_Clean_BUILD_Release.sh [TARGET_ECUx] [Release|Debug]
# If no ECU is specified, all 4 are built with --skip-import optimization.
# Config defaults to Release if not specified.

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"

ECU_ARG=""
CONF="Release"
for arg in "$@"; do
	case "$arg" in
		TARGET_*) ECU_ARG="$arg" ;;
		Release|Debug) CONF="$arg" ;;
	esac
done

if [ -e /workspace ]; then
	set -e
	if [ -n "$ECU_ARG" ]; then
		bash "${SCRIPT_DIR}/_build_ecu.sh" "$ECU_ARG" "$CONF"
	else
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUA "$CONF"
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUB "$CONF" --skip-import
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUC "$CONF" --skip-import
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUD "$CONF" --skip-import
	fi
	exit 0
fi

STM32CUBEIDEWORKSPACE="$( cd "${SCRIPT_DIR}/../../" ; pwd )"
DOCKER_STM32CUBEIDE_VERSION=7.0 # see https://github.com/xanderhendriks/action-build-stm32cubeide#stm32-cube-ide-versions

( cd "${STM32CUBEIDEWORKSPACE}"; docker run --rm -v .:/workspace xanderhendriks/stm32cubeide:${DOCKER_STM32CUBEIDE_VERSION} /workspace/scripts/build/docker_Clean_BUILD_Release.sh $@ ) || exit 1
