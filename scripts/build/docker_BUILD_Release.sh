#!/bin/bash
# Incremental-build ECU(s) with the STM32CubeIDE Docker container.
# Usage: docker_BUILD_Release.sh [TARGET_ECUx] [Release|Debug]
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
	OUTPUT_FOLDER="/workspace/scripts/firmware"
	PROJECT_NAME=RAMNV1
	mkdir -p "${OUTPUT_FOLDER}"
	if [ -n "$ECU_ARG" ]; then
		ECU_NAME="${ECU_ARG#TARGET_}"
		bash "${SCRIPT_DIR}/_build_ecu.sh" "$ECU_ARG" "$CONF" --no-clean
		cp -pr "/workspace/firmware/${PROJECT_NAME}/${CONF}/${PROJECT_NAME}.hex" "${OUTPUT_FOLDER}/${ECU_NAME}.hex"
	else
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUA "$CONF" --no-clean
		cp -pr "/workspace/firmware/${PROJECT_NAME}/${CONF}/${PROJECT_NAME}.hex" "${OUTPUT_FOLDER}/ECUA.hex"
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUB "$CONF" --skip-import --no-clean
		cp -pr "/workspace/firmware/${PROJECT_NAME}/${CONF}/${PROJECT_NAME}.hex" "${OUTPUT_FOLDER}/ECUB.hex"
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUC "$CONF" --skip-import --no-clean
		cp -pr "/workspace/firmware/${PROJECT_NAME}/${CONF}/${PROJECT_NAME}.hex" "${OUTPUT_FOLDER}/ECUC.hex"
		bash "${SCRIPT_DIR}/_build_ecu.sh" TARGET_ECUD "$CONF" --skip-import --no-clean
		cp -pr "/workspace/firmware/${PROJECT_NAME}/${CONF}/${PROJECT_NAME}.hex" "${OUTPUT_FOLDER}/ECUD.hex"
	fi
	exit 0
fi

STM32CUBEIDEWORKSPACE="$( cd "${SCRIPT_DIR}/../../" ; pwd )"
DOCKER_STM32CUBEIDE_VERSION=7.0 # see https://github.com/xanderhendriks/action-build-stm32cubeide#stm32-cube-ide-versions

( cd "${STM32CUBEIDEWORKSPACE}"; docker run --rm -v .:/workspace xanderhendriks/stm32cubeide:${DOCKER_STM32CUBEIDE_VERSION} /workspace/scripts/build/docker_BUILD_Release.sh $@ ) || exit 1
