cmake_minimum_required(VERSION 2.8)
project(ecto_samples)

ectomodule(hello_ecto
    hello_ecto.cpp
)

add_subdirectory(tutorial)
