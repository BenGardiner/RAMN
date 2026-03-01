#!/bin/bash
# Inner build script — runs inside the STM32CubeIDE Docker container.
# Usage: _build_ecu.sh TARGET_ECUx [Release|Debug] [--skip-import] [--no-clean]

ECU="${1:?Usage: $0 TARGET_ECUx [Release|Debug] [--skip-import] [--no-clean]}"
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

PROJECT_NAME=RAMNV1
PROJECT_WORKSPACE="/workspace/firmware/${PROJECT_NAME}"

set -e

if [ "$SKIP_IMPORT" = false ]; then
	stm32cubeide --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /tmp/stm-workspace -import ${PROJECT_WORKSPACE}
fi

# Normalize file timestamps to avoid "Clock skew detected" warnings from make
# when the Docker container's clock differs from the host that created the files.
find /workspace -type f -exec touch -c {} + 2>/dev/null || true

headless-build.sh -data /tmp/stm-workspace ${BUILD_MODE} ${PROJECT_NAME}/${PROJECT_CONF} -D ${ECU}
