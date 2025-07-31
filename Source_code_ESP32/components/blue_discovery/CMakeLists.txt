set(pri_req nvs_flash driver log)

idf_component_register(SRCS "blutooth_scan.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_req} bt)
