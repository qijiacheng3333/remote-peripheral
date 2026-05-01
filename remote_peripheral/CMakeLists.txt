cmake_minimum_required(VERSION 3.10)
project(remote_peripheral C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

include_directories(include)

add_library(remote_peripheral STATIC
    src/mcu_protocol.c
    src/mcu_transport.c
    src/remote_gpio.c
    src/remote_i2c.c
    src/remote_spi.c
    src/remote_pwm.c
)

enable_testing()

foreach(t protocol transport gpio i2c spi pwm)
    add_executable(test_${t} tests/test_${t}.c)
    target_link_libraries(test_${t} remote_peripheral)
    add_test(NAME test_${t} COMMAND test_${t})
endforeach()
