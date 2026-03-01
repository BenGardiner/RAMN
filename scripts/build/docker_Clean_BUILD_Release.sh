#!/bin/bash
# Clean-build all 4 ECUs (Release) by delegating to the single-ECU script.

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"

for ECU in TARGET_ECUA TARGET_ECUB TARGET_ECUC TARGET_ECUD; do
	bash "${SCRIPT_DIR}/docker_Clean_BUILD_Release_ECU.sh" "${ECU}" Release || exit 1
done
