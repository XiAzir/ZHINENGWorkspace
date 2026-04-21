idf_component_register(
    SRCS
        "app_main.c"
        "app_freertos.c"
        "tasks/task_audio.c"
        "tasks/task_doa_feature.c"
        "tasks/task_ai.c"
        "tasks/task_decision.c"
        "tasks/task_gui.c"
        "tasks/task_gui_fast.c"
        "tasks/task_orientation.c"
        "tasks/task_haptic.c"
        "tasks/task_ble.c"
        "tasks/task_adaptive.c"
    INCLUDE_DIRS "."
    REQUIRES dsp_kernel ai_wrapper drivers freertos log esp_timer
)

# WAV 仿真数据嵌入（取消注释并放置对应 WAV 文件后生效）
# target_add_binary_data(${COMPONENT_LIB} "test_audio_car_horn.wav"   BINARY)
# target_add_binary_data(${COMPONENT_LIB} "test_audio_siren.wav"      BINARY)
# target_add_binary_data(${COMPONENT_LIB} "test_audio_background.wav" BINARY)
