#version 400 core

const int TYPE_UNLIT = 0;
const int TYPE_PBR = 1;

uniform int type;
uniform bool hasTexture;
uniform vec3 lightInWorld, cameraInWorld, lightColor, objectColor;
uniform sampler2D textureSampler;

in vec3 positionInWorld, normalInWorld;
in vec2 uv;

out vec4 fragmentColor;

void main() {
    vec3 baseColor = hasTexture ? texture(textureSampler, uv).rgb : objectColor;
    switch (type) {
        case TYPE_UNLIT:
            fragmentColor = vec4(baseColor, 1.0);
            break;
        case TYPE_PBR:
            // Ambient
            float ambientStrength = 0.6;
            vec3 ambient = ambientStrength * lightColor;
            // Diffuse
            vec3 lightDirInWorld = normalize(lightInWorld - positionInWorld);
            float diff = max(dot(normalInWorld, lightDirInWorld), 0.0);
            vec3 diffuse = diff * lightColor;
            // Specular
            float specularStrength = 0.5;
            vec3 viewDirection = normalize(cameraInWorld - positionInWorld);
            vec3 reflectDirection = reflect(-lightDirInWorld, normalInWorld);
            float spec = pow(max(dot(viewDirection, reflectDirection), 0.0), 32);
            vec3 specular = specularStrength * spec * lightColor;
            // Combination
            vec3 result = (ambient + diffuse + specular) * baseColor;
            fragmentColor = vec4(result, 1.0);
            break;
        default:
            // Magenta for error (like Source engine :D)
            fragmentColor = vec4(1.0, 0.0, 1.0, 1.0);
            break;
    }
}
