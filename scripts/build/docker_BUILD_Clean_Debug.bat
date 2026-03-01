@echo off
call %~dp0_version.bat
cd %~dp0\..\..\
docker run --rm -v .:/workspace xanderhendriks/stm32cubeide:%DOCKER_STM32CUBEIDE_VERSION% /workspace/scripts/build/docker_BUILD_Clean_Debug.sh
