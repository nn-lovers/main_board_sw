# Add executable. Default name is the project name, version 0.1

add_executable(w5100_test main.cpp )

pico_set_program_name(w5100_test "w5100_test")
pico_set_program_version(w5100_test "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(w5100_test 0)
pico_enable_stdio_usb(w5100_test 1)

# Add the standard library to the build
target_link_libraries(w5100_test
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(w5100_test PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${PORT_DIR}/freertos
)

# Add any user requested libraries
target_link_libraries(w5100_test 
        pico_stdlib
        hardware_timer
        FreeRTOS-Kernel-Heap4
        IOLIBRARY_FILES
        ETHERNET_FILES
        W5100S_FILES
        TIMER_FILES
        )

pico_add_extra_outputs(w5100_test)

file(GLOB binary_files
        ${CMAKE_CURRENT_BINARY_DIR}/*.bin
        ${CMAKE_CURRENT_BINARY_DIR}/*.uf2
        ${CMAKE_CURRENT_BINARY_DIR}/*.hex
        ${CMAKE_CURRENT_BINARY_DIR}/*.elf
        ${CMAKE_CURRENT_BINARY_DIR}/*.dis
)
add_custom_command(
    TARGET w5100_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${binary_files}
        ${CMAKE_BINARY_DIR}
)