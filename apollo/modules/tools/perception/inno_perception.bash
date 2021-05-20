#!/usr/bin/env bash

cd "$(dirname "${BASH_SOURCE[0]}")"

source "/apollo/scripts/apollo_base.sh"

/apollo/bazel-bin/modules/tools/perception/replay_perception inno_*.json
