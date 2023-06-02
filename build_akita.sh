#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

exec tools/bazel run --config=akita --config=fast //private/devices/google/akita:zuma_akita_dist "$@"
