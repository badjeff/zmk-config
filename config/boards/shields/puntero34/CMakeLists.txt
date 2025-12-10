# zephyr_library()


#---
# patch pat912x driver to NOT reset on startup
#---
set(FILE_PATH "${APPLICATION_SOURCE_DIR}/../zephyr/drivers/input/input_pat912x.c")
file(READ "${FILE_PATH}" FILE_CONTENT)
# message(STATUS "File path: ${FILE_PATH}")
# message(STATUS "File contents:\n${FILE_CONTENT}")
string(
  REPLACE "\ti2c_reg_write_byte_dt(&cfg->i2c, PAT912X_CONFIGURATION, CONFIGURATION_RESET);\n"
          "\t\/\/ i2c_reg_write_byte_dt(&cfg->i2c, PAT912X_CONFIGURATION, CONFIGURATION_RESET);\n"
  FILE_CONTENT "${FILE_CONTENT}"
)
# message(STATUS "File contents:\n${FILE_CONTENT}")
file(WRITE "${FILE_PATH}" "${FILE_CONTENT}")
message(STATUS "Patched PAT912x driver: ${FILE_PATH}")


# zephyr_include_directories(include)
