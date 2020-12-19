#ifndef SIM_TEXTURE_H
#define SIM_TEXTURE_H

#include <unordered_map>

#include <glad/glad.h>
#include <spdlog/spdlog.h>


class Texture {
public:
    inline static GLuint getTexture(std::unordered_map<std::string, GLuint>& textures, std::string& filePath) {
        if(textures.find(filePath) != textures.end()) return textures[filePath];

        GLuint ID;
        glGenTextures(1, &ID);

        int width, height, numChannels;
        stbi_set_flip_vertically_on_load(true);
        stbi_uc* data = stbi_load(filePath.c_str(), &width, &height, &numChannels, 4);
        if(!data) SPDLOG_ERROR("Unable to load texture at " + filePath);

        glBindTexture(GL_TEXTURE_2D, ID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
        textures[filePath] = ID;
        return ID;
    }
};

#endif //SIM_TEXTURE_H
