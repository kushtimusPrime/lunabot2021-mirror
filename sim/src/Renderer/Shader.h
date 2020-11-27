#ifndef SIM_SHADER_H
#define SIM_SHADER_H

#include <fstream>
#include <sstream>
#include <string>

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>

class Shader {
public:
    inline explicit Shader(const std::string& name) {
        std::ifstream vFile;
        std::ifstream fFile;
        vFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        vFile.open("assets/shaders/" + name + ".vert");
        fFile.open("assets/shaders/" + name + ".frag");
        std::stringstream vStream, fStream;
        vStream << vFile.rdbuf();
        fStream << fFile.rdbuf();
        vFile.close();
        fFile.close();

        std::string vCode = vStream.str();
        std::string fCode = fStream.str();
        const char* vCodeC = vCode.c_str();
        const char* fCodeC = fCode.c_str();

        int success;
        char infoLog[512];

        GLuint vID = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vID, 1, &vCodeC, nullptr);
        glCompileShader(vID);
        glGetShaderiv(vID, GL_COMPILE_STATUS, &success);
        if(!success) {
            glGetShaderInfoLog(vID, 512, nullptr, infoLog);
            SPDLOG_ERROR("Vertex Shader {} failed to compile: {}", name, infoLog);
        }

        GLuint fID = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fID, 1, &fCodeC, nullptr);
        glCompileShader(fID);
        glGetShaderiv(fID, GL_COMPILE_STATUS, &success);
        if(!success) {
            glGetShaderInfoLog(fID, 512, nullptr, infoLog);
            SPDLOG_ERROR("Fragment Shader {} failed to compile: {}", name, infoLog);
        }

        ID = glCreateProgram();
        glAttachShader(ID, vID);
        glAttachShader(ID, fID);
        glLinkProgram(ID);
        glGetProgramiv(ID, GL_LINK_STATUS, &success);
        if(!success) {
            glGetShaderInfoLog(ID, 512, nullptr, infoLog);
            SPDLOG_ERROR("Shader {} failed to link: {}", name, infoLog);
        }

        glDeleteShader(vID);
        glDeleteShader(fID);
    }
    inline ~Shader() { glDeleteProgram(ID); }

    inline void use() const { glUseProgram(ID); };

    // Uniforms
    inline void setFloat(const char* name, float value) const {
        glUniform1f(glGetUniformLocation(ID, name), value);
    }
    inline void setInt(const char* name, int value) const {
        glUniform1i(glGetUniformLocation(ID, name), value);
    }
    inline void setMat4(const char* name, const glm::mat4& value) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name), 1, GL_FALSE, glm::value_ptr(value));
    }
    inline void setVec3(const char* name, const glm::vec3& value) const {
        glUniform3fv(glGetUniformLocation(ID, name), 1, glm::value_ptr(value));
    }
    inline void setVec4(const char* name, const glm::vec4& value) const {
        glUniform4fv(glGetUniformLocation(ID, name), 1, glm::value_ptr(value));
    }
private:
    GLuint ID;
};

#endif //SIM_SHADER_H
