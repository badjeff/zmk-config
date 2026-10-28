# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

target_sources(app PRIVATE input_processor_mixer.c)

zephyr_include_directories(include)
