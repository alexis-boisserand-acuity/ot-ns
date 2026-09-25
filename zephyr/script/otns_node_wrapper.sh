#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# OTNS node launcher wrapper for a Zephyr native_sim executable.
#
# OTNS launches each simulated node with positional arguments:
#     <executable> <nodeId> <socketPath> [<randomSeed>]
#
# The Zephyr native_sim command-line parser does not accept bare positional
# arguments, so this wrapper translates them into the --otns-* options that the
# OTNS IEEE 802.15.4 driver registers, and gives every node its own flash file.
#
# Point OTNS at this wrapper as the node executable, for example inside OTNS:
#     > exe /absolute/path/to/otns_node_wrapper.sh
# or via the OTNS ExecutableConfig, then:
#     > add router
#
# The path to the built zephyr.exe can be overridden with $ZEPHYR_EXE.

set -eu

NODE_ID="${1:?missing node id}"
SOCKET="${2:?missing socket path}"
SEED="${3:-}"

# Location of the native_sim executable. Adjust or export ZEPHYR_EXE as needed.
# Default assumes a `west build -b native_sim coap_server` run from the west
# workspace top directory (produces <topdir>/build/zephyr/zephyr.exe).
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ZEPHYR_EXE="${ZEPHYR_EXE:-$SCRIPT_DIR/../../../../build/zephyr/zephyr.exe}"

# Per-node flash file so nodes do not share persisted settings.
FLASH_DIR="${OTNS_FLASH_DIR:-/tmp/otns-zephyr}"
mkdir -p "$FLASH_DIR"
FLASH="$FLASH_DIR/node_${NODE_ID}.flash"

# Seed the native_sim entropy device uniquely per node. OpenThread derives its
# random extended (MAC) address and other identifiers from this entropy at first
# boot; without a per-node seed every node process reuses the same default seed
# and generates an identical extended address, so the nodes collide and can
# never merge into a single Thread network. OTNS supplies a unique random seed
# per node; fall back to the node id when it is absent to keep nodes distinct.
ENTROPY_SEED="${SEED:-$NODE_ID}"

set -- "$ZEPHYR_EXE" \
        --otns-node-id="$NODE_ID" \
        --otns-socket="$SOCKET" \
        --flash="$FLASH" \
        --seed="$ENTROPY_SEED"

if [ -n "$SEED" ]; then
	set -- "$@" --otns-seed="$SEED"
fi

exec "$@"
