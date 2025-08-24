cmake_minimum_required(VERSION 3.10.0)
project(strapdown VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Define source files
set(SOURCES
    src/process_strapdown_integration.c
    src/Quaternion.c
    src/strapdown_integration.c
    src/helper.c
)

# Create executable
add_executable(strapdown ${SOURCES})

# Add include directory (modern way)
target_include_directories(strapdown PRIVATE include)

# Link math library
target_link_libraries(strapdown PRIVATE m)
