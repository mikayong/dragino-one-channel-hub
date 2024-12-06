set(liblorahub "lorahub_aux.c" "lorahub_hal.c")

idf_component_register(SRCS "${liblorahub}"
                       REQUIRES esp_timer driver
                       INCLUDE_DIRS ".")
