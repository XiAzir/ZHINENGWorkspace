idf_component_register(
    SRCS
        "drv_pdm_dma.c"
        "drv_oled.c"
        "drv_i2c_bus.c"
        "drv_haptic.c"
        "drv_imu.c"
        "drv_ble.c"
        "drv_sim.c"
    INCLUDE_DIRS "."
    REQUIRES driver log freertos esp_timer
)

target_add_binary_data(${COMPONENT_LIB} "test_audio_car_horn.wav"   BINARY)
target_add_binary_data(${COMPONENT_LIB} "test_audio_siren.wav"      BINARY)
target_add_binary_data(${COMPONENT_LIB} "test_audio_background.wav" BINARY)
