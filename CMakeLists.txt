if(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_INERTIA)
    zephyr_library()
    zephyr_library_sources(src/input_processor_scroll_inertia.c)
endif()
