add_library(renderer glad.c Renderer.cpp Renderer.h Window.h Object.h Camera.cpp Camera.h Shader.h Texture.h)

target_link_libraries(renderer dl glfw)
