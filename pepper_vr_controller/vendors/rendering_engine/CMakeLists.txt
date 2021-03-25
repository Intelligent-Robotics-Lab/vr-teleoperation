
##### GLEW #####
add_definitions(-DGLEW_STATIC)
execute_process(COMMAND make WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/vendors/glew/auto")
add_subdirectory(vendors/glew/build/cmake)
################

##### GLFW #####
add_subdirectory(vendors/glfw)
################

##### GLM  #####
add_subdirectory(vendors/glm)
################

#### OpenVR ####
add_subdirectory(vendors/openvr)
################

#### Assimp ####
add_subdirectory(vendors/assimp)
################

#### ImGui #####
add_subdirectory(vendors/imgui)
################


add_library(rendering_engine    src/Application.cpp
                                src/Window.cpp
                                src/PerspectiveCamera.cpp
                                src/Input.cpp
                                src/PerspectiveCameraController.cpp
                                src/VRSystem.cpp
                                src/VRHmd.cpp
                                src/VRCamera.cpp
                                src/VRInput.cpp
                                src/Mesh.cpp
                                src/Model.cpp
                                src/Utilities.cpp
                                src/Canvas.cpp
                                src/ReferenceFrame.cpp
                                src/Renderer/Renderer.cpp
                                src/Renderer/Framebuffer.cpp
                                src/Renderer/Texture2D.cpp
                                src/Renderer/Material.cpp
                                src/Renderer/Shader.cpp
                                src/WindowOverlay.cpp
                                src/GUI.cpp)

target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/glew/include")
target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/glfw/include")
target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/stb")
target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/glm")
target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/openvr/headers")
target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/assimp/include")
target_include_directories(rendering_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/imgui")


target_link_libraries(rendering_engine PRIVATE glew glfw openvr_api assimp imgui)