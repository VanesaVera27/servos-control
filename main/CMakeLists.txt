idf_component_register(SRCS "app_main.cpp"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi freertos
                    INCLUDE_DIRS ".")
