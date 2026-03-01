#!/bin/bash
# Build a single ECU with the STM32CubeIDE Docker image.
# Usage: docker_Clean_BUILD_Release_ECU.sh TARGET_ECUx

ECU="${1:?Usage: $0 TARGET_ECUx}"
ECU_NAME="${ECU#TARGET_}"

OUTPUT_FOLDER="/workspace/scripts/firmware"
PROJECT_NAME=RAMNV1
PROJECT_CONF=Release
PROJECT_WORKSPACE="/workspace/firmware/${PROJECT_NAME}"

if [ -e /workspace ]; then
	set -e
	mkdir -p "${OUTPUT_FOLDER}"

	stm32cubeide --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /tmp/stm-workspace -import ${PROJECT_WORKSPACE}

	headless-build.sh -data /tmp/stm-workspace -cleanBuild ${PROJECT_NAME}/${PROJECT_CONF} -D ${ECU}
	cp -pr "${PROJECT_WORKSPACE}/${PROJECT_CONF}/${PROJECT_NAME}.hex" "${OUTPUT_FOLDER}/${ECU_NAME}.hex"

	exit 0
fi

STM32CUBEIDEWORKSPACE="$( cd "$(dirname "${BASH_SOURCE[0]}")"/../../ ; pwd )"
DOCKER_STM32CUBEIDE_VERSION=7.0 # see https://github.com/xanderhendriks/action-build-stm32cubeide#stm32-cube-ide-versions
DOCKER_STM32CUBEIDE_COMMAND="docker run --rm -v .:/workspace xanderhendriks/stm32cubeide:${DOCKER_STM32CUBEIDE_VERSION} /workspace/scripts/build/docker_Clean_BUILD_Release_ECU.sh ${ECU}"

( cd "${STM32CUBEIDEWORKSPACE}"; ${DOCKER_STM32CUBEIDE_COMMAND} ) || exit 1
