
# Testing Shared Memory on CycloneDDS over network

## Build & Run the Tests

Beforehand please delete all build and install folders from your workspace, Docker will
build the network test from scratch inside of the container.
This is only done initially to create the local docker image. After that the build inside the image is reused to run the network tests.

Simply run the script: `run_network_test.sh`

You will be given an option to choose to build and run the tests, after which the necessary Docker containers will be built and run automatically.

## Setting up your own system

For help setting up your own network test, refer to the entrypoint scripts used by the example docker containers.
