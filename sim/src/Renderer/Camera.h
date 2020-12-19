#ifndef SIM_CAMERA_H
#define SIM_CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Window.h"

class Camera {
public:
    const float zNear = 0.1f;
    const float zFar = 100.0f;
    const float fov = glm::radians(45.0f);

    explicit Camera(Window& window);

    void update();
    void setCursorPos(double x, double y);
    inline void setCursorPosCallback() {
        glfwGetCursorPos(window.ptr, &prevX, &prevY);
        glfwSetCursorPosCallback(window.ptr, [](GLFWwindow* window, double x, double y) {
            ((Camera*)glfwGetWindowUserPointer(window))->setCursorPos(x, y);
        });
    }
    inline void enable() {
        enabled = true;
        glfwSetInputMode(window.ptr, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    };
    inline void disable() {
        enabled = false;
        glfwSetInputMode(window.ptr, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    };
    inline void setProjection(int width, int height) {
        window.width = width;
        window.height = height;

        uniformBuffer.projection = glm::perspective(fov,
                                                    (float)window.width / window.height, zNear, zFar);
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(uniformBuffer.projection),
                        glm::value_ptr(uniformBuffer.projection));
    }
    inline void setView(const glm::mat4& view) {
        uniformBuffer.view = view;
        glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), &uniformBuffer.view);
    }
    inline const glm::mat4& getView() {
        return uniformBuffer.view;
    }
    inline float getFOV() const { return fov; }
    // Y and Z are swapped
    inline glm::vec3 getPos() const { return glm::vec3(pos.x, pos.z, pos.y); }
    inline glm::vec3 getRot() const { return glm::vec3(0, glm::radians(yaw), glm::radians(pitch)); }
private:
    struct UniformBuffer {
        glm::mat4 projection = glm::identity<glm::mat4>();
        glm::mat4 view = glm::identity<glm::mat4>();
    };

    Window& window;

    GLuint UBO = 0;
    UniformBuffer uniformBuffer;

    glm::vec3 pos{ 2.439393, 0.354034, 7.124863 };
    glm::vec3 up{ 0, 1, 0 };
    glm::vec3 front{0, 0, 0};

    double prevTime = glfwGetTime();
    const float baseSpeed = 1;
    const double sensitivity = 0.1;

    double yaw = -102.1;
    double pitch = 0.7;
    double prevX, prevY;
    bool enabled = true;
};

#endif //SIM_CAMERA_H
