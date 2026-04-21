idf_component_register(
    SRCS "ai_wrapper.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp-tflite-micro esp_partition log freertos
)

target_add_binary_data(${COMPONENT_LIB} "audiosense_model.tflite" TEXT)
