idf_component_register(
    SRCS
        "main.c"
        "dynamixel_ax12a.c"
    INCLUDE_DIRS "."
    REQUIRES driver
    PRIV_REQUIRES esp_timer
)