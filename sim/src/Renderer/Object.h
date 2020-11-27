#ifndef SIM_OBJECT_H
#define SIM_OBJECT_H

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "Shader.h"

struct Material {
    glm::vec3 ambientColor;
    glm::vec3 diffuseColor;
    glm::vec3 specularColor;
    float shininess;

    bool hasDiffuseTex;

    GLuint diffuseTex;

    Material(const glm::vec3 &ambientColor, const glm::vec3 &diffuseColor, const glm::vec3 &specularColor,
             float shininess, GLuint diffuseTex) :
            ambientColor(ambientColor), diffuseColor(diffuseColor), specularColor(specularColor),
            shininess(shininess), hasDiffuseTex(diffuseTex != -1), diffuseTex(diffuseTex) {}

    void setUniforms(const std::string &materialName, const Shader &shader) const {
        std::string ambientColorName = materialName + ".ambientColor";
        std::string diffuseColorName = materialName + ".diffuseColor";
        std::string specularColorName = materialName + ".specularColor";
        std::string shininessName = materialName + ".shininess";
        std::string hasDiffuseTexName = materialName + ".hasDiffuseTex";
        std::string diffuseTexName = materialName + ".diffuseTex";

        shader.setVec3(ambientColorName.c_str(), ambientColor);
        shader.setVec3(diffuseColorName.c_str(), diffuseColor);
        shader.setVec3(specularColorName.c_str(), specularColor);
        shader.setInt(hasDiffuseTexName.c_str(), hasDiffuseTex ? 1 : 0);
        shader.setFloat(shininessName.c_str(), shininess);

        glActiveTexture(GL_TEXTURE0);
        shader.setInt(diffuseTexName.c_str(), 0);
        if(hasDiffuseTex) {
            glBindTexture(GL_TEXTURE_2D, diffuseTex);
        } else {
            // TODO: Use a unique for unknown texture
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
};

class Object {
public:
    struct Vertex {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec2 texCoord;
        glm::vec3 color;

        Vertex(const glm::vec3 &pos, const glm::vec3 &normal, const glm::vec2 &texCoord, const glm::vec3 &color) :
                pos(pos), normal(normal), texCoord(texCoord), color(color) {}
    };

    inline void addVertices(const std::unordered_map<int, std::vector<Vertex>>& vertices) {
        for (const auto& [materialId, materialVertices] : vertices) {
            this->vertices[materialId].insert(this->vertices[materialId].end(), materialVertices.begin(), materialVertices.end());
        }
    };

    inline void bufferVertices() {
        for (const auto& [materialId, materialVertices] : vertices) {
            glCreateBuffers(1, &VBOs[materialId]);
            glNamedBufferStorage(VBOs[materialId], sizeof(Vertex) * materialVertices.size(), materialVertices.data(), 0);
        }
    }

    inline void addMaterial(Material material) { this->materials.emplace_back(material); };

    inline void render(GLuint VAO, const Shader &shader) const {
        shader.use();
        for (const auto&[materialId, VBO] : VBOs) {
            materials[materialId].setUniforms("material", shader);
            glVertexArrayVertexBuffer(VAO, 0, VBO, 0, sizeof(Vertex));
            glDrawArrays(GL_TRIANGLES, 0, vertices.at(materialId).size());
        }
    };
private:
    std::vector<Material> materials;

    std::unordered_map<int, std::vector<Vertex>> vertices; // Vertices organized by material
    std::unordered_map<int, GLuint> VBOs; // VBO associated with material id
};


#endif //SIM_OBJECT_H
