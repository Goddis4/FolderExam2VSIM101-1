#ifndef BALLPHYS_H
#define BALLPHYS_H

#include <glm/glm.hpp>
#include <iostream>
#include "Datasett.h"

class Ballphys;

struct CollisionInfo {
    bool hasCollision;
    float time;
    glm::vec3 collisionNormal;
};


class Ballphys {
public:
    //Constructor
    Ballphys(float mass, float radius, float gravity, float deltaTime);

    
    void setPosition(const glm::vec3& position);
    void setVelocity(const glm::vec3& velocity);

    
    glm::vec3 getPosition() const;
    glm::vec3 getVelocity() const;
    float getMass() const;
    float getRadius() const;

    
    glm::vec3 calculateAcceleration(const glm::vec3& normal, float gravity) const;

    
    void updatePosition(const Datasett& datasett);

    // Collision detection and response
    float detectCollision(const Ballphys& other) const;
    void resolveCollision(Ballphys& other);
    
    

private:
    // Physics properties
    glm::vec3 mPosition;
    glm::vec3 mVelocity;
    glm::vec3 mAngularVelocity;
    float mMass;
    float mRadius;
    float mGravity;
    float mDeltaTime;
    bool mStopped;

    
    void handleSurfaceInteraction(const Datasett& datasett);
};




#endif
