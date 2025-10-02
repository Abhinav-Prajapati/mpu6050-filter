# Add our main application executable
add_executable(${NAME}
    main.cpp
)

# Link the necessary libraries
target_link_libraries(${NAME}
    pico_stdlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    freertos_config       # Your project's FreeRTOS configuration
)

# Standard Pico setup for USB/UART output and creating a UF2 file
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
pico_add_extra_outputs(${NAME})