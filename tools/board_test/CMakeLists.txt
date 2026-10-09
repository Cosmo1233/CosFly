# CosFly V1 board bring-up test: a standalone ESP-IDF project, separate from
# the flight firmware in the repository root. See README.md.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(CosFlyBoardTest)
