cmake_minimum_required(VERSION 2.8)

project(ecto_sample)

find_package(ecto REQUIRED)

ectomodule(sample sample.cpp)

