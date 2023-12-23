#version 410 core

uniform vec3 lightInWorld, cameraInWorld, lightColor, objectColor;

in vec3 positionInWorld, normalInWorld;

out vec4 fragmentColor;

void main() {
    // Ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;
    // Diffuse
    vec3 normal = normalize(normalInWorld);
    vec3 lightDirection = normalize(lightInWorld - positionInWorld);
    float diff = max(dot(normal, lightDirection), 0.0);
    vec3 diffuse = diff * lightColor;
    // Specular
    float specularStrength = 0.5;
    vec3 viewDirection = normalize(cameraInWorld - positionInWorld);
    vec3 reflectDirection = reflect(-lightDirection, normal);
    float spec = pow(max(dot(viewDirection, reflectDirection), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;
    // Combination
    vec3 result = (ambient + diffuse + specular) * objectColor;
    fragmentColor = vec4(result, 1.0);
}
