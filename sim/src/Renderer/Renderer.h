#ifndef SIM_RENDERER_H
#define SIM_RENDERER_H

#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Camera.h"
#include "Object.h"
#include "Shader.h"
#include "Window.h"

class Renderer {
public:
    Renderer(const std::string& name, int width, int height);
    inline bool isRunning() { return !window.shouldClose(); }
    std::pair<std::vector<std::uint8_t>, std::vector<float>> render(const Shader& shader); // Returns the RGB-D
    inline void update() {
        camera.update();
        glfwPollEvents();
    }

    bool loadObject(const std::string& objectName);
    float getZNear() const { return camera.zNear; }
    float getZFar() const { return camera.zFar; }
    inline int getWidth() const { return window.getWidth(); }
    inline int getHeight() const { return window.getHeight(); }
    inline float getFOV() const { return camera.getFOV(); }
private:
    Window window;
    Camera camera; // Must be below window for correct initialization order
    std::vector<Object> objects;
    std::unordered_map<std::string, GLuint> textures;

    double prevFPSTime = glfwGetTime();
    unsigned int framesPassed = 0;

    GLuint VAO = 0;
};


#endif //SIM_RENDERER_H
