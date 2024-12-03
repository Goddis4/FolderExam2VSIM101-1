#include "BSplineSurface.h"
#include <GLFW/glfw3.h>  
#include <glad/glad.h>   
#include <glm/glm.hpp>   
#include <iostream>



BSplineSurface::BSplineSurface()
    : n_u(4), n_v(3), d_u(2), d_v(2) {
    // Initialize knot vectors (clamped)
    mu = { 0, 0, 0, 1, 2, 2, 2 };
    mv = { 0, 0, 0, 1, 1, 1 };

    // Initialize control points (4x3 grid)
    mc = {
        glm::vec3(0, 0, 0), glm::vec3(1, 0, 0), glm::vec3(2, 0, 0), glm::vec3(3, 0, 0),
        glm::vec3(0, 1, 0), glm::vec3(1, 1, 2), glm::vec3(2, 1, 2), glm::vec3(3, 1, 0),
        glm::vec3(0, 2, 0), glm::vec3(1, 2, 0), glm::vec3(2, 2, 0), glm::vec3(3, 2, 0)
    };

    generateSurfacePoints();

    initBuffers();
}

float BSplineSurface::BasisFunction(int k, const std::vector<float>& knots, int i, float t) {
    if (k == 0) {
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0f : 0.0f;
    }

    float leftWeight = 0.0f;
    if (knots[i + k] - knots[i] != 0.0f) {
        leftWeight = (t - knots[i]) / (knots[i + k] - knots[i]);
    }
    float leftBasis = leftWeight * BasisFunction(k - 1, knots, i, t);

    float rightWeight = 0.0f;
    if (knots[i + k + 1] - knots[i + 1] != 0.0f) {
        rightWeight = (knots[i + k + 1] - t) / (knots[i + k + 1] - knots[i + 1]);
    }
    float rightBasis = rightWeight * BasisFunction(k - 1, knots, i + 1, t);

    return leftBasis + rightBasis;
}

void BSplineSurface::initBuffers() {
    // Create and bind surface VAO
    glGenVertexArrays(1, &surfaceVAO);
    glBindVertexArray(surfaceVAO);

    // Create surface VBO for vertices
    glGenBuffers(1, &surfaceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, surfaceVBO);
    glBufferData(GL_ARRAY_BUFFER, mVertices.size() * sizeof(glm::vec3), mVertices.data(), GL_STATIC_DRAW);

    // Enable vertex attribute for position (assuming it's at location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    // Create IBO for indices
    unsigned int ibo;
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Unbind surface VAO
    glBindVertexArray(0);
}



glm::vec3 BSplineSurface::evaluateBiquadratic(int i, int j, float u, float v) {
    glm::vec3 result(0.0f);

    // Iterate over the control points affected by the basis functions
    for (int vIndex = 0; vIndex <= d_v; ++vIndex) { // Control points in v-direction
        for (int uIndex = 0; uIndex <= d_u; ++uIndex) { // Control points in u-direction
            int cp_j = j - d_v + vIndex;
            int cp_i = i - d_u + uIndex;

            if (cp_j >= 0 && cp_j < n_v && cp_i >= 0 && cp_i < n_u) {
                int index = cp_j * n_u + cp_i;
                float basisU = BasisFunction(d_u, mu, cp_i, u);
                float basisV = BasisFunction(d_v, mv, cp_j, v);
                float basisValue = basisU * basisV;
                result += basisValue * mc[index];
               
            }
        }
    }
    return result;
}


int BSplineSurface::findKnotInterval(const std::vector<float>& knots, int degree, int numControlPoints, float t) {
    // Knot span is found by checking the knot vector
    int n = numControlPoints + degree + 1;  // Number of knots
    for (int i = degree; i < n - degree - 1; ++i) {
        if (t >= knots[i] && t < knots[i + 1]) {
            
            return i;  // Return the knot interval
        }
    }

    
    return n - degree - 2;
}



void BSplineSurface::generateIndices(int num_u, int num_v) {
    indices.clear();

    for (int i = 0; i < num_v - 1; ++i) {
        for (int j = 0; j < num_u - 1; ++j) {
            // Vertex indices of a quad (two triangles)
            int topLeft = i * num_u + j;
            int topRight = topLeft + 1;
            int bottomLeft = (i + 1) * num_u + j;
            int bottomRight = bottomLeft + 1;
            

            // First triangle of the quad
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            // Second triangle of the quad
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }
}


void BSplineSurface::generateSurfacePoints() {
    float h = 0.1f;  // Step size
    int num_u = static_cast<int>((mu.back() - mu[0]) / h) + 1;
    int num_v = static_cast<int>((mv.back() - mv[0]) / h) + 1;

    mVertices.clear();
    

    // Iterate over u and v using the knot vectors
    for (float v = mv[d_v]; v < mv[n_v + 1]; v += h) {  
        for (float u = mu[d_u]; u < mu[n_u + 1]; u += h) {  
            // Find knot intervals for u and v
            int i = findKnotInterval(mu, d_u, n_u, u);
            int j = findKnotInterval(mv, d_v, n_v, v);  

            // Calculate point on the surface
            glm::vec3 point = evaluateBiquadratic(i, j, u, v);
            mVertices.push_back(point);

           
        }
    }

    generateIndices(num_u, num_v);
}


void BSplineSurface::renderWireframe() {
    
    // Set OpenGL to wireframe mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Bind the surface VAO
    glBindVertexArray(surfaceVAO);

    // Draw the wireframe
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    // Unbind the VAO
    glBindVertexArray(0);

    // Reset OpenGL to normal polygon fill mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}
