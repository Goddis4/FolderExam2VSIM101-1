#pragma once
#include <vector>
#include <glm/glm.hpp>

class BSplineSurface {
private:
    std::vector<float> mu, mv;  // Knot vectors
    std::vector<glm::vec3> mc;  // Control points

    unsigned int surfaceVAO, surfaceVBO;
    std::vector<glm::vec3> mVertices;  // Surface vertices
    std::vector<unsigned int> indices;  // Indices for rendering
    unsigned int vao;  // Vertex array object

    float BasisFunction(int k, const std::vector<float>& knots, int i, float t);  // Recursive basis function
    glm::vec3 evaluateBiquadratic(int my_u, int my_v, float u, float v);  // Surface point evaluation

    int findKnotInterval(const std::vector<float>& knots, int degree, int numControlPoints, float t);  // Knot interval finding

public:
    BSplineSurface();
    void generateSurfacePoints();
    void renderWireframe();
    void initBuffers();
    void generateIndices(int num_u, int num_v);
    int n_u, n_v, d_u, d_v;
};


