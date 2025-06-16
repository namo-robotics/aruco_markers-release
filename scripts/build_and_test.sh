#!/bin/bash

set -eo pipefail

DIR=$(dirname "$0")
cd $DIR/..

colcon build
colcon test --event-handlers console_direct+ --return-code-on-test-failure --parallel-workers 1