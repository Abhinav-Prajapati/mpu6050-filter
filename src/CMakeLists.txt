add_executable(${NAME}
    main.cpp
)

# Link the necessary libraries
target_link_libraries(${NAME}
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel-Heap4 
    freertos_config     
)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
pico_add_extra_outputs(${NAME})