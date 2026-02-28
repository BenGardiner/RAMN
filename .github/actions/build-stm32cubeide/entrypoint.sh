#!/bin/bash

# Import the project into the workspace
# $1 is intentionally unquoted so that values like "firmware/RAMNV1 -D TARGET_ECUB"
# are word-split into separate CLI arguments for stm32cubeide.
stm32cubeide --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data workspace -import $1

# Build the specific target
# $2 is intentionally unquoted for the same reason.
headless-build.sh -data workspace -build $2
