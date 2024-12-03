#ifndef SPHERES_H
#define SPHERES_H

#include <glm/glm.hpp>
#include <vector>
#include "Shader.h"
#include "Ballphys.h"
#include "Datasett.h"
#include "Ballphys.h"
class Spheres {
public:
    
    Spheres(float radius, const glm::vec3& initialPosition = glm::vec3(0.0f));

    
    void createSphere();
    void renderSphere(Shader& shader, const glm::mat4& view, const glm::mat4& projection);
    void updateSphere(const Datasett& surface);

    Ballphys mPhysics;

private:
    unsigned int mVAO, mVBO; 
    glm::vec3 mPosition;     
    std::vector<glm::vec3> mVertices; 
    float mRadius;           

    
    void lagTriangel(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);
    void subDivide(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, int n);
};

#endif


