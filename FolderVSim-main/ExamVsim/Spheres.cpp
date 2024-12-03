#include "Spheres.h"
#include <glad/glad.h>
#include <iostream>

Spheres::Spheres(float radius, const glm::vec3& initialPosition)
    : mRadius(radius),
    mPhysics(1.0f,  // masse
        radius,  // radius 
        9.81f,  // gravitasjon
        0.016f)  // delta time
{
    mPhysics.setPosition(initialPosition);
    createSphere();
}


void Spheres::createSphere() {
    // Generate vertices for an icosahedron-based sphere
    glm::vec3 v0{ 0.0f, 0.0f, 1.0f };
    glm::vec3 v1{ 1.0f, 0.0f, 0.0f };
    glm::vec3 v2{ 0.0f, 1.0f, 0.0f };
    glm::vec3 v3{ -1.0f, 0.0f, 0.0f };
    glm::vec3 v4{ 0.0f, -1.0f, 0.0f };
    glm::vec3 v5{ 0.0f, 0.0f, -1.0f };

    subDivide(v0, v1, v2, 4);
    subDivide(v0, v2, v3, 4);
    subDivide(v0, v3, v4, 4);
    subDivide(v0, v4, v1, 4);
    subDivide(v5, v2, v1, 4);
    subDivide(v5, v3, v2, 4);
    subDivide(v5, v4, v3, 4);
    subDivide(v5, v1, v4, 4);

    // Setup VAO and VBO for rendering
    glGenVertexArrays(1, &mVAO);
    glGenBuffers(1, &mVBO);

    glBindVertexArray(mVAO);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);

    if (mVertices.empty()) {
        std::cerr << "Error: No vertices for sphere" << std::endl;
        return;
    }

    glBufferData(GL_ARRAY_BUFFER, mVertices.size() * sizeof(glm::vec3), mVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Spheres::renderSphere(Shader& shader, const glm::mat4& view, const glm::mat4& projection) {
    shader.use();

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, mPhysics.getPosition()); // Position fra fysikk klasse
    model = glm::scale(model, glm::vec3(mRadius)); 

    shader.setMat4("model", model);
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);

    shader.setBool("useFlatColor", true); // Toggle flat color fra shader
    shader.setVec3("objectColor", glm::vec3(1.0f, 0.0f, 0.0f)); // Farge til baller

    glBindVertexArray(mVAO);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(mVertices.size()));
    glBindVertexArray(0);

    shader.setBool("useFlatColor", false);
}


void Spheres::updateSphere(const Datasett& surface) {
    mPhysics.updatePosition(surface);

}




void Spheres::subDivide(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, int n) {
    if (n > 0) {
        glm::vec3 v1 = glm::normalize(a + b);
        glm::vec3 v2 = glm::normalize(a + c);
        glm::vec3 v3 = glm::normalize(c + b);

        subDivide(a, v1, v2, n - 1);
        subDivide(c, v2, v3, n - 1);
        subDivide(b, v3, v1, n - 1);
        subDivide(v3, v2, v1, n - 1);
    }
    else {
        lagTriangel(a, b, c);
    }
}

void Spheres::lagTriangel(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3) {
    mVertices.push_back(v1);
    mVertices.push_back(v2);
    mVertices.push_back(v3);
}
