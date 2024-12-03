#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;
in float Friction;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec3 lightColor;
uniform vec3 objectColor;
uniform bool useFlatColor; 

void main() {
    if (useFlatColor) {
        // Use flat color without lighting
        FragColor = vec4(objectColor, 1.0);
        return;
    }

    if (Friction > 0.5) {
        // Render with a flat color for friction zones
        FragColor = vec4(0.0, 0.0, 1.0, 1.0); // Greenish color for friction zones
        return;
    }

    // Phong shading
    // Ambient lighting
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse lighting
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular lighting
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;

    // Resulting color
    vec3 result = (ambient + diffuse + specular) * objectColor;
    FragColor = vec4(result, 1.0);
}
