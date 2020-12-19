#ifndef SIM_WINDOW_H
#define SIM_WINDOW_H

#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

void APIENTRY glDebugCallback(GLenum source, GLenum mType, GLuint id, GLenum severity, GLsizei length,
                              const GLchar* message, const void* userParam);

class Window {
public:
    inline Window(const std::string& name, int width, int height) : name(name), width(width), height(height) {
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        ptr = glfwCreateWindow(width, height, name.c_str(), nullptr, nullptr);
        if(ptr == nullptr) throw std::runtime_error("Failed to create window!");

        glfwMakeContextCurrent(ptr);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            throw std::runtime_error("Failed to initialize GLAD!");
        }

#ifdef _DEBUG
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glDebugCallback, nullptr);
        GLuint ignoredIDs[] = { 0x20071 };
        glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_OTHER, GL_DONT_CARE, 1, ignoredIDs, GL_FALSE);
        glfwSetErrorCallback([](int errorCode, const char* desc) {
            SPDLOG_CRITICAL("GLFW Error {}: {}", errorCode, desc);
        });
#endif

        glEnable(GL_DEPTH_TEST);
    }

    inline ~Window() {
        glfwTerminate();
    }

    inline bool shouldClose() {
        return glfwWindowShouldClose(ptr);
    }

    inline void swapBuffers() {
        glfwSwapBuffers(ptr);
    }

    inline void setFPS(double fps) {
        std::string windowTitle = name + " - " + std::to_string(fps) + " FPS";
        glfwSetWindowTitle(ptr, windowTitle.c_str());
    }

    inline int getWidth() const { return width; }
    inline int getHeight() const { return height; }

    friend class Camera;
private:
    GLFWwindow* ptr;

    std::string name;
    int width;
    int height;
};

inline void APIENTRY glDebugCallback(GLenum source, GLenum mType, GLuint id, GLenum severity, GLsizei length,
                                     const GLchar* message, const void* userParam) {
    std::string src;
    switch(source) {
        case GL_DEBUG_SOURCE_API_ARB:
            src = "Windows";
            break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER_ARB:
            src = "Shader Compiler";
            break;
        case GL_DEBUG_SOURCE_THIRD_PARTY_ARB:
            src = "Third Party";
            break;
        case GL_DEBUG_SOURCE_APPLICATION_ARB:
            src = "Application";
            break;
        case GL_DEBUG_SOURCE_OTHER_ARB:
            src = "Other";
            break;
    }

    std::string type;
    switch(mType) {
        case GL_DEBUG_TYPE_ERROR_ARB:
            type = "Error";
            break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR_ARB:
            type = "Deprecated Behavior";
            break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR_ARB:
            type = "Undefined Behavior";
            break;
        case GL_DEBUG_TYPE_PORTABILITY_ARB:
            type = "Portability Error";
            break;
        case GL_DEBUG_TYPE_PERFORMANCE_ARB:
            type = "Performance Error";
            break;
        case GL_DEBUG_TYPE_OTHER_ARB:
            type = "Other Message";
            break;
    }

    switch(severity) {
        case GL_DEBUG_SEVERITY_LOW_ARB:
            SPDLOG_WARN("GL LOW - {} {}: {}", src, type, message);
            break;
        case GL_DEBUG_SEVERITY_MEDIUM_ARB:
            SPDLOG_ERROR("GL MEDIUM - {} {}: {}", src, type, message);
            break;
        case GL_DEBUG_SEVERITY_HIGH_ARB:
            SPDLOG_CRITICAL("GL HIGH - {} {}: {}", src, type, message);
            break;
    }
}

#endif //SIM_WINDOW_H
