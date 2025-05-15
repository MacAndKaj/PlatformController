#!/bin/bash
echo "Running $0 in $(pwd)"

#parse arguments - [--skip-build, --skip-tests, --output-only]
SKIP_BUILD=false
SKIP_TESTS=false
OUTPUT_ONLY=false
for arg in "$@"
do
    case $arg in
        --skip-build)
            SKIP_BUILD=true
            ;;
        --skip-tests)
            SKIP_TESTS=true
            ;;
        --output-only)
            OUTPUT_ONLY=true
            ;;
        *)
            echo "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

# if no output only
if [ "$OUTPUT_ONLY" = false ]; then

    # if not skip build
    if [ "$SKIP_BUILD" = false ]; then
        # Build the package
        echo "Building the package"
        colcon build --cmake-args -DBUILD_TESTS=ON --packages-select motoros_interfaces platform_controller

        # if previous command fails, exit
        if [ $? -ne 0 ]; then
            echo "Build failed"
            exit 1
        fi
    fi
    # if not skip tests
    if [ "$SKIP_TESTS" = false ]; then
        # Run tests
        echo "Running tests"
        colcon test --packages-select platform_controller

        # if previous command fails, exit
        if [ $? -ne 0 ]; then
            echo "Running tests failed"
            exit 1
        fi
    fi
fi
# examine test results

echo "Examining test results"
colcon test-result --all --verbose 
# if previous command fails, exit
if [ $? -ne 0 ]; then
    echo "Examining test results failed"
    exit 1
fi