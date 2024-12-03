#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <glm/glm.hpp>


class Datasett {
public:
    
    std::vector<glm::vec3> points;       // Original 3D points
    std::vector<size_t> indices;         // Triangle indices
    std::vector<double> delaunayCoords;  // Flattened 2D points for Delaunator
    std::vector<glm::vec3> normals;      // Per-vertex normals
    GLuint VBO, VAO, EBO, normalVBO;     // OpenGL buffers

    // Member functions
    void loadFile(const std::string& filename);                 
    void triangulate();                                         
    void calculateNormals();                                    
    
    //Kalkulasjon
    glm::vec3 computeBarycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const;
    glm::vec3 getSurfaceNormalAt(const glm::vec3& position) const;
    glm::vec3 calculateClosestPointOnTriangle(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const;
    float calculateHeightAtPoint(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const;


    
    float getHeightAt(const glm::vec3& position) const;
    void setupBuffers(const std::vector<glm::vec3>& points,const std::vector<size_t>& triangles);

    
    //friksjon
    bool isInFrictionZone(size_t triangleIndex) const;
    void assignFrictionCoefficients();
    float getFrictionCoefficientAt(const glm::vec3& position) const;

private:
    std::vector<float> frictionCoefficients;

};

