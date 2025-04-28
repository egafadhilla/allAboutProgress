idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES 
                    mdns
                    esp_http_server
                    esp_https_ota
                    )