if(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_INERTIA)
    target_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_INERTIA app PRIVATE src/input_processor_scroll_inertia.c)
endif()
