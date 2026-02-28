#!/bin/bash

# Import the project into the workspace
stm32cubeide --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data workspace -import "$1"

# Build the specific target
headless-build.sh -data workspace -build "$2"
