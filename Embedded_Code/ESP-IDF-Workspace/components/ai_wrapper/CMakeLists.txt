idf_component_register(
    SRCS "ai_wrapper.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp-dl esp_partition log esp_timer freertos
)
