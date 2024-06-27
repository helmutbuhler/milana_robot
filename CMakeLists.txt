# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make

#############
# Prepare cuda for asr_server on WSL Linux:
#############
# Allow more RAM for WSL here: "C:\Users\*username*\.wslconfig":
# [wsl2]
# memory=30GB
# Install cuda in WSL2:
# https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local

#############
# Prepare cuda for asr_server on real Linux:
#############
# install nvidia driver and cuda-toolkit: https://www.cherryservers.com/blog/install-cuda-ubuntu
# sudo apt install nvidia-cuda-toolkit nvidia-cuda-toolkit-gcc

#############
# Set CUDA_ARCHITECTURES
#############
# You also need to adjust the CUDA_ARCHITECTURES number here (during the set_property call below):
# First find out your gpu capability by running:
# nvidia-smi --query-gpu=compute_cap --format=csv
# Then enter that number without the dot after CUDA_ARCHITECTURES.

cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

set(CMAKE_CUDA_COMPILER /usr/local/cuda-12.1/bin/nvcc)

project(asr_server LANGUAGES CXX CUDA)
add_executable(asr_server
	asr/asr_server.cpp
	llm/chat.cpp

	common/helper.cpp
	common/network.cpp
	common/time_helper.cpp

	3rdparty/whisper.cpp/whisper.cpp
	3rdparty/llama.cpp/ggml.c
	3rdparty/llama.cpp/ggml-alloc.c
	3rdparty/llama.cpp/ggml-backend.c
	3rdparty/llama.cpp/ggml-cuda.cu
	3rdparty/llama.cpp/ggml-quants.c
	3rdparty/llama.cpp/llama.cpp
	)
target_compile_definitions(asr_server PUBLIC _GNU_SOURCE GGML_USE_CUBLAS)
set_property(TARGET asr_server PROPERTY CUDA_ARCHITECTURES 75)
set_target_properties(asr_server PROPERTIES CXX_STANDARD 14)
target_link_libraries(asr_server cublas)


project(control_ui)
include_directories(3rdparty/imgui)
find_package(glm REQUIRED)
add_executable(control_ui
	3rdparty/imgui/imgui.cpp
	3rdparty/imgui/imgui_demo.cpp
	3rdparty/imgui/imgui_draw.cpp
	3rdparty/imgui/imgui_impl_glfw.cpp
	3rdparty/imgui/imgui_impl_opengl2.cpp
	3rdparty/imgui/imgui_tables.cpp
	3rdparty/imgui/imgui_widgets.cpp

	3rdparty/implot/implot.cpp
	3rdparty/implot/implot_demo.cpp
	3rdparty/implot/implot_items.cpp

	3rdparty/jpeg/jpgd.cpp
	3rdparty/jpeg/jpge.cpp

	common/geometry.cpp
	common/helper.cpp
	common/leaning.cpp
	common/network.cpp
	common/time_helper.cpp

	control_ui/control_ui.cpp
	control_ui/control_ui_client.cpp
	control_ui/pendulum_simulation.cpp
	control_ui/plot.cpp
	control_ui/simulation_ui.cpp
	control_ui/video_recorder.cpp
	control_ui/webcam.cpp
	)
target_compile_definitions(control_ui PUBLIC COMPILING_CONTROL_UI)
if (CMAKE_COMPILER_IS_GNUCC)
	target_compile_options(control_ui PRIVATE -Wfloat-conversion)
endif()
target_link_libraries(control_ui glfw GL glm::glm)


project(robot)
add_executable(robot
	common/geometry.cpp
	common/helper.cpp
	common/leaning.cpp
	common/network.cpp
	common/time_helper.cpp

	common/odrive/ODrive.cpp
	common/odrive/endpoint.cpp

	robot/imu/sensor_fusion.cpp
	robot/imu/spi_imu.cpp

	robot/asr_client.cpp
	robot/balance_control.cpp
	robot/battery.cpp
	robot/body_acc.cpp
	robot/command.cpp
	robot/foot_control.cpp
	robot/ik.cpp
	robot/joystick.cpp
	robot/leg_control.cpp
	robot/main.cpp
	robot/pin.cpp
	robot/server.cpp
	robot/servo.cpp
	robot/statistics.cpp
	robot/tts_client.cpp
	)
if (CMAKE_COMPILER_IS_GNUCC)
	target_compile_options(robot PRIVATE -Wfloat-conversion)
endif()
target_link_libraries(robot pthread usb-1.0 asound)
