# SPDX-FileCopyrightText: Copyright (c) 2022-2024 Oğuz Toraman <oguz.toraman@tutanota.com>
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.14)

project(picar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER g++)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/inc")

file(GLOB SRCS "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_executable(picar main.cpp ${SRCS})

add_compile_options(-Wall -Werror)

target_link_libraries(picar ${OpenCV_LIBS} pthread)
