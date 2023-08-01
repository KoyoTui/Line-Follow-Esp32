idf_component_register(SRCS "Buzzer.c"
                    INCLUDE_DIRS "include" "../../include" "${IDF_PATH}/components/driver/include"
                    REQUIRES driver esp_event esp_adc)
