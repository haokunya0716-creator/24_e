cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME QD4310)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}



)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths

)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

        Application/App_Button/app_button.c
        Application/App_QD4310_PID/app_QD4310_PID.c
        Application/App_Usartptint/app_usart.c
        Application/App_Task/app_task.c
        Application/App_Emm_V5/app_Emm_V5.c
        Application/App_roboarm/app_roboarm.c
        Application/App_Solenoid/app_solenoid.c
        Application/App_Solenoid/app_solenoid.h

        Libraries/Button/button.c
        Libraries/Delay/delay.c
        Libraries/PID/pid.c
        Libraries/QD4310/QD4310.c
        Libraries/Emm_V5/Emm_V5.c


        Test/button_test.c
        Test/button_test.h
        Test/buzzer_test.c
        Test/buzzer_test.h
        Test/delay_test.c
        Test/delay_test.h


       )

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/Inc
        Application
        Application/App_Button
        Application/App_QD4310_PID
        Application/App_Usartptint
        Application/App_Solenoid
        Application/App_Emm_V5


        Libraries
        Libraries/Button
        Libraries/Delay
        Libraries/PID
        Libraries/QD4310


        Test
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)
target_link_options(${PROJECT_NAME} PRIVATE
        -u _printf_float
)
