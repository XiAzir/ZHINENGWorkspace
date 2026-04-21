idf_component_register(
    SRCS
        "dsp_common.c"
        "dsp_filter.c"
        "dsp_mel.c"
        "mel_filterbank.c"
        "dsp_gcc_phat.c"
        "dsp_noise_tracker.c"
        "dsp_threat_tracker.c"
    INCLUDE_DIRS "."
    REQUIRES esp-dsp esp_common log
)
