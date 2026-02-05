cmake_minimum_required(VERSION 3.15)
project(foc_dash CXX)

find_package(asio REQUIRED)
find_package(glfw3 REQUIRED)
find_package(fmt REQUIRED)
find_package(glad REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(foc_dash src/main.cpp src/gui.cpp)

add_library(
    imgui
    src/imgui/imgui_demo.cpp
    src/imgui/imgui_draw.cpp
    src/imgui/imgui_tables.cpp
    src/imgui/imgui_widgets.cpp
    src/imgui/imgui.cpp
    src/imgui/imgui_impl_glfw.cpp
    src/imgui/imgui_impl_opengl3.cpp
)

add_library(
    implot
    src/implot/implot.cpp
    src/implot/implot_demo.cpp
    src/implot/implot_items.cpp
)

target_include_directories(foc_dash PRIVATE src)
target_include_directories(imgui PUBLIC src/imgui)
target_include_directories(implot PUBLIC src/implot)

target_link_libraries(imgui glfw)
target_link_libraries(implot imgui)
target_link_libraries(foc_dash asio::asio glfw fmt::fmt glad::glad implot imgui opengl::opengl)

install(
    TARGETS foc_dash
    DESTINATION "."
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
