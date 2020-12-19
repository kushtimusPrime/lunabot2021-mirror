#version 330

in vec2 texCoord;

struct Material {
	vec3 ambientColor;
	vec3 diffuseColor;
	vec3 specularColor;
	float shininess;

	int hasDiffuseTex;

	sampler2D diffuseTex;
};
uniform Material material;

out vec4 FragColor;

void main() {
	vec4 ambient = vec4(material.ambientColor, 1);

	vec4 diffuse;
	if(material.hasDiffuseTex != 0) {
		diffuse = texture(material.diffuseTex, texCoord);
	} else {
		diffuse = vec4(material.diffuseColor, 1);
	}

	FragColor = ambient * 0.1 + diffuse * 0.9;
}