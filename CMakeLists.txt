cmake_minimum_required(VERSION 3.22)


# Setup compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for  C CXX ASM languages
enable_language(C CXX ASM)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# disable some warnings
set(COM_FLAGS "-Wno-unused-parameter -Wno-missing-field-initializers -Wno-pedantic -Wno-unknown-pragmas -Wno-comment")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COM_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COM_FLAGS} -Wno-reorder")

# ########################## USER CONFIG SECTION ##############################
# Set the project name
set(CMAKE_PROJECT_NAME milFOC)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# ====================== 用户自定义文件夹配置 ======================
# USER_FOLDERS: 一级用户文件夹列表
# 每个一级文件夹下支持两种结构:
#   1. 直接包含 src/ inc/ 子目录 (传统结构)
#   2. 包含多个二级模块子目录 (如 CAN/, USART/),
#      每个模块子目录下可有 Src/ Inc/ 或 src/ inc/ 或文件直接放置
# =================================================================
set(USER_FOLDERS "BSP" "Modules" "App" "Resources")

# 创建列表存放用户源文件和头文件目录
set(USER_SOURCES "")
set(USER_INCLUDE_DIRS "")

# 辅助宏: 扫描指定目录下的源文件并追加到 USER_SOURCES
macro(collect_sources base_dir)
    file(GLOB_RECURSE found_sources
        "${base_dir}/*.c"
        "${base_dir}/*.s"
        "${base_dir}/*.cpp"
    )
    if(found_sources)
        list(APPEND USER_SOURCES ${found_sources})
    endif()
endmacro()

# 辅助宏: 扫描指定目录下的头文件并将其所在目录追加到 USER_INCLUDE_DIRS
macro(collect_headers base_dir)
    file(GLOB_RECURSE found_headers
        "${base_dir}/*.h"
        "${base_dir}/*.hpp"
    )
    foreach(hdr ${found_headers})
        get_filename_component(hdr_dir ${hdr} DIRECTORY)
        list(APPEND USER_INCLUDE_DIRS ${hdr_dir})
    endforeach()
endmacro()

foreach(folder ${USER_FOLDERS})
    set(cur_dir "${CMAKE_SOURCE_DIR}/${folder}")

    # --- 1. 传统一级 src/ inc/ 扫描 ---
    if(EXISTS "${cur_dir}/src")
        collect_sources("${cur_dir}/src")
    endif()
    if(EXISTS "${cur_dir}/inc")
        collect_headers("${cur_dir}/inc")
    endif()

    # --- 2. 一级目录下的直接文件 (如 MotorControl/*.cpp) ---
    collect_sources("${cur_dir}")
    collect_headers("${cur_dir}")

    # --- 3. 二级模块子目录扫描 (如 Resources/CAN/, Resources/USART/) ---
    file(GLOB module_dirs LIST_DIRECTORIES true "${cur_dir}/*")
    foreach(mod ${module_dirs})
        if(IS_DIRECTORY ${mod})
            # 跳过已处理的一级 src/inc 目录
            get_filename_component(mod_name ${mod} NAME)
            if(mod_name STREQUAL "src" OR mod_name STREQUAL "inc"
               OR mod_name STREQUAL "Src" OR mod_name STREQUAL "Inc")
                continue()
            endif()

            # 优先扫描子目录下的 Src/ 和 Inc/ (大小写兼容)
            if(EXISTS "${mod}/Src")
                collect_sources("${mod}/Src")
            endif()
            if(EXISTS "${mod}/src")
                collect_sources("${mod}/src")
            endif()
            if(EXISTS "${mod}/Inc")
                collect_headers("${mod}/Inc")
            endif()
            if(EXISTS "${mod}/inc")
                collect_headers("${mod}/inc")
            endif()

            # 同时扫描模块子目录下的直接文件 (兼容无 Src/Inc 的情况)
            collect_sources("${mod}")
            collect_headers("${mod}")
        endif()
    endforeach()
endforeach()

# 去重
if(USER_SOURCES)
    list(REMOVE_DUPLICATES USER_SOURCES)
endif()
if(USER_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)
endif()

######################################################################################

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${USER_INCLUDE_DIRS})

add_subdirectory(cmake/stm32cubemx)
