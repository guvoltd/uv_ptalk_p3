# Animated GIF Library
# https://github.com/bitbank2/AnimatedGIF.git
# License: Copyright (c) 2020-2024 BitBank Software, Inc.

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "src/AnimatedGIF.cpp"
                       INCLUDE_DIRS "src"
                       REQUIRES arduino
)

project(AnimatedGIF)