#! /usr/bin/env bash
# SPDX-License-Identifier: zlib-acknowledgement

[[ ! -d x11-build ]] && mkdir x11-build

gcc -DAPP_DEV_BUILD -g code/x11-app.c -o x11-build/x11-app -lX11 -lXrandr -lXrender
