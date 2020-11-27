#version 330

layout (std140) uniform Camera {
	uniform mat4 projection;
	uniform mat4 view;
};

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in vec3 aColor;

out vec2 texCoord;

void main() {
	texCoord = aTexCoord;
	gl_Position = projection * view * vec4(aPos, 1);
}