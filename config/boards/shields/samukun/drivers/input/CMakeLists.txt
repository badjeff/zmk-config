# SPDX-License-Identifier: MIT

zephyr_library()

target_sources(app PRIVATE input_mlx90393.c)

zephyr_include_directories(include)
