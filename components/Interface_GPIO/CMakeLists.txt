idf_component_register(SRCS "Interface_GPIO.c"
                      INCLUDE_DIRS "include" "../../include" "${IDF_PATH}/components/driver/include"
                      REQUIRES esp_event driver esp_adc)
