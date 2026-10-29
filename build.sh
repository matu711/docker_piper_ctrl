#!/bin/sh
set -eu

PIPER_CTRL_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
docker build "$@" -t "${PIPER_IMAGE:-piper-control:noetic}" "${PIPER_CTRL_DIR}"
