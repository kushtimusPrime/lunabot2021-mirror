#include "Renderer.h"

#include <unordered_map>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "Texture.h"

bool Renderer::loadObject(const std::string& objectName) {
    spdlog::set_level(spdlog::level::debug);

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    std::string scenePath = "assets/scenes/" + objectName + "/";
    std::string objFilePath = scenePath + objectName + ".obj";
    spdlog::debug("Reading OBJ and MTL Files");
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                objFilePath.c_str(), scenePath.c_str(), true);
    spdlog::debug("Read OBJ and MTL Files");

    if(!warn.empty()) SPDLOG_WARN(warn);
    if(!err.empty()) SPDLOG_ERROR(err);

    if(!ret) return false;

    Object object;
    spdlog::debug("Loading {} Meshes", shapes.size());
    for(const auto& shape : shapes) {
        std::unordered_map<int, std::vector<Object::Vertex>> vertices;
        const auto& mesh = shape.mesh;
        // All faces should be triangles because of the triangulate parameter
        for(const auto& numVertices: mesh.num_face_vertices) { assert(numVertices == 3); }

        auto it = mesh.indices.begin();
        for(const auto& materialId : mesh.material_ids) {
            for(int i = 0; i < 3; i++) {
                const auto& posI = 3 * it->vertex_index;
                const auto& normalI = 3 * it->normal_index;
                const auto& texCoordI = 2 * it->texcoord_index;
                const auto& colorI = 3 * it->vertex_index;
                // TODO: Avoid vertex duplication
                vertices[materialId].emplace_back(
                    glm::vec3(attrib.vertices[posI + 0], attrib.vertices[posI + 1],
                              attrib.vertices[posI + 2]),
                    glm::vec3(attrib.normals[normalI + 0], attrib.normals[normalI + 1],
                              attrib.normals[normalI + 2]),
                    glm::vec2(attrib.texcoords[texCoordI + 0], attrib.texcoords[texCoordI + 1]),
                    glm::vec3(attrib.colors[colorI + 0], attrib.colors[colorI + 1],
                              attrib.colors[colorI + 2])
                );
                it++;
            }
        }

        object.addVertices(vertices);
    }
    object.bufferVertices();
    spdlog::debug("Loaded {} Meshes", shapes.size());

    spdlog::debug("Loading {} Materials", materials.size());
    for(const auto& material : materials) {
        GLuint diffuseTex = -1;
        if(!material.diffuse_texname.empty()) {
            std::string diffusePath = scenePath + material.diffuse_texname;
            std::replace(diffusePath.begin(), diffusePath.end(), '\\', '/');
            diffuseTex = Texture::getTexture(textures, diffusePath);
        }
        object.addMaterial(Material(
            glm::vec3(material.ambient[0], material.ambient[1], material.ambient[2]),
            glm::vec3(material.diffuse[0], material.diffuse[1], material.diffuse[2]),
            glm::vec3(material.specular[0], material.specular[1], material.specular[2]),
            material.shininess,
            diffuseTex
        ));
    }
    spdlog::debug("Loaded {} Materials", materials.size());

    objects.emplace_back(std::move(object));
    spdlog::debug("Added Object");
    return true;
}

std::pair<std::vector<std::uint8_t>, std::vector<float>> Renderer::render(const Shader& shader) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBindVertexArray(VAO);
    for(const auto& object : objects) {
        object.render(VAO, shader);
    }

    int width = window.getWidth();
    int height = window.getHeight();
    std::vector<std::uint8_t> colorBuffer(width * height * 3);
    std::vector<float> depthBuffer(width * height);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, colorBuffer.data());
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer.data());
    // Flip vertically since (0, 0) is bottom left corner
    for(int line = 0; line < height / 2; line++) {
        for(int pixel = 0; pixel < width; pixel++) {
            std::swap(depthBuffer[line * width + pixel], depthBuffer[(height - 1 - line) * width + pixel]);
            for(int i = 0; i < 3; i++) {
                std::swap(colorBuffer[3 * (line * width + pixel) + i], colorBuffer[3 * ((height - 1 - line) * width + pixel) + i]);
            }
        }
    }

    window.swapBuffers();
    framesPassed++;
    if(glfwGetTime() - prevFPSTime >= 1) {
        double curTime = glfwGetTime();
        window.setFPS(framesPassed / (curTime - prevFPSTime));
        prevFPSTime = curTime;
        framesPassed = 0;
    }

    return {colorBuffer, depthBuffer};
}

Renderer::Renderer(const std::string &name, int width, int height) : window(name, width, height), camera(window) {
    glCreateVertexArrays(1, &VAO);

    glEnableVertexArrayAttrib(VAO, 0);
    glEnableVertexArrayAttrib(VAO, 1);
    glEnableVertexArrayAttrib(VAO, 2);
    glEnableVertexArrayAttrib(VAO, 3);

    glVertexArrayAttribFormat(VAO, 0, 3, GL_FLOAT, GL_FALSE, offsetof(Object::Vertex, Object::Vertex::pos));
    glVertexArrayAttribFormat(VAO, 1, 3, GL_FLOAT, GL_FALSE, offsetof(Object::Vertex, Object::Vertex::normal));
    glVertexArrayAttribFormat(VAO, 2, 2, GL_FLOAT, GL_FALSE, offsetof(Object::Vertex, Object::Vertex::texCoord));
    glVertexArrayAttribFormat(VAO, 3, 3, GL_FLOAT, GL_FALSE, offsetof(Object::Vertex, Object::Vertex::color));

    glVertexArrayAttribBinding(VAO, 0, 0);
    glVertexArrayAttribBinding(VAO, 1, 0);
    glVertexArrayAttribBinding(VAO, 2, 0);
    glVertexArrayAttribBinding(VAO, 3, 0);
}
