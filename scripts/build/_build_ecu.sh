#!/bin/bash
# Inner build script — runs inside the STM32CubeIDE Docker container.
# Usage: _build_ecu.sh TARGET_ECUx [Release|Debug] [--skip-import] [--no-clean]

ECU="${1:?Usage: $0 TARGET_ECUx [Release|Debug] [--skip-import] [--no-clean]}"
ECU_NAME="${ECU#TARGET_}"
PROJECT_CONF="${2:-Release}"
shift 2 2>/dev/null || shift $#

SKIP_IMPORT=false
BUILD_MODE="-cleanBuild"
for arg in "$@"; do
	case "$arg" in
		--skip-import) SKIP_IMPORT=true ;;
		--no-clean) BUILD_MODE="-build" ;;
	esac
done

OUTPUT_FOLDER="/workspace/scripts/firmware"
PROJECT_NAME=RAMNV1
PROJECT_WORKSPACE="/workspace/firmware/${PROJECT_NAME}"

set -e
mkdir -p "${OUTPUT_FOLDER}"

if [ "$SKIP_IMPORT" = false ]; then
	stm32cubeide --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /tmp/stm-workspace -import ${PROJECT_WORKSPACE}
fi

headless-build.sh -data /tmp/stm-workspace ${BUILD_MODE} ${PROJECT_NAME}/${PROJECT_CONF} -D ${ECU}
cp -pr "${PROJECT_WORKSPACE}/${PROJECT_CONF}/${PROJECT_NAME}.hex" "${OUTPUT_FOLDER}/${ECU_NAME}.hex"
