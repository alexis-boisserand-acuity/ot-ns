#!/bin/sh
# Copyright (c) 2026, The OpenThread Authors.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# OTNS node launcher wrapper for a Zephyr native_sim executable.

set -eu

NODE_ID="${1:?missing node id}"
SOCKET="${2:?missing socket path}"
SEED="${3:-}"

ZEPHYR_EXE="${ZEPHYR_EXE:?ZEPHYR_EXE must be set to the built zephyr.exe path}"

OTNS_DATA_PATH="${OTNS_DATA_PATH:?OTNS_DATA_PATH must be set by OTNS}"
PORT_OFFSET="${PORT_OFFSET:?PORT_OFFSET must be set by OTNS}"
FLASH="${OTNS_DATA_PATH}/${PORT_OFFSET}_${NODE_ID}.flash"

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
