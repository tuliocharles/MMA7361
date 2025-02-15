idf_component_register(SRCS "MMA7361.c"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        driver
                        esp_adc)
