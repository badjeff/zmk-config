
zephyr_library()

target_sources(app PRIVATE behavior_battery_printer.c)

zephyr_include_directories(include)
