cmake_minimum_required(VERSION 3.12)
project(path_planner)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# Encontra o Python e PyBind11
find_package(Python REQUIRED COMPONENTS Development NumPy)
find_package(pybind11 REQUIRED)

# Biblioteca principal C++
add_library(path_planner_lib STATIC 
    src/path_planner.cpp
)

# Módulo Python
pybind11_add_module(path_planner 
    path_planner_pybind.cpp
)

# Linka a biblioteca principal ao módulo Python
target_link_libraries(path_planner PRIVATE 
    path_planner_lib
    pybind11::module
)

# Configurações para C++17
set_target_properties(path_planner PROPERTIES 
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)