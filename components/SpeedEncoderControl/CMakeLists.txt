idf_component_register(SRCS "SpeedEncoderControl.c"
                    INCLUDE_DIRS "include" "../../include" "${IDF_PATH}/components/driver/include"
                    REQUIRES esp_event driver esp_adc) 
