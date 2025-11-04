
zephyr_library()

if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources(app PRIVATE behavior_battery_printer.c)
endif()

zephyr_include_directories(include)
