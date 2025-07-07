idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c
                    REQUIRES spiffs)

spiffs_create_partition_image(storage ../spiffs_log FLASH_IN_PROJECT)