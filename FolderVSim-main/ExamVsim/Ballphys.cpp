#include "Ballphys.h"
#include "Datasett.h"

Ballphys::Ballphys(float mass, float radius, float gravity, float deltaTime)
    : mMass(mass)
    , mRadius(radius)
    , mGravity(gravity)
    , mDeltaTime(deltaTime)
    , mStopped(false)
    , mPosition(0.0f)
    , mVelocity(0.0f)
{
}

void Ballphys::setPosition(const glm::vec3& position) {
    mPosition = position;
}

void Ballphys::setVelocity(const glm::vec3& velocity) {
    mVelocity = velocity;
}

glm::vec3 Ballphys::getPosition() const {
    return mPosition;
}

glm::vec3 Ballphys::getVelocity() const {
    return mVelocity;
}

float Ballphys::getMass() const {
    return mMass;
}

float Ballphys::getRadius() const {
    return mRadius;
}

//9.14 fra boka. hvordan regne akselerasjonsvektor. blir ikke brukt
glm::vec3 Ballphys::calculateAcceleration(const glm::vec3& normal, float gravity) const {
    float nx = normal.x;
    float ny = normal.y;
    float nz = normal.z;
    return gravity * glm::vec3(nx * nz, ny * nz, nz * nz - 1.0f);
    
}




// Oppdaterer ballens posisjon og hastighet basert på gravitasjon underlag. 
// Stopper ballen hvis den er i ro. Kaller på handleSurfaceInteraction() for å sjekke terrenginteraksjon.
void Ballphys::updatePosition(const Datasett& datasett) {
    if (mStopped) return;

    glm::vec3 acceleration = glm::vec3(0.0f, -mGravity, 0.0f);
    mVelocity += acceleration * mDeltaTime;
    mPosition += mVelocity * mDeltaTime;

    
    handleSurfaceInteraction(datasett);
}


// Beregner tid til kollisjon mellom to baller og returnerer tid (0.0f - 1.0f)
float Ballphys::detectCollision(const Ballphys& other) const {
    glm::vec3 relativePosition = mPosition - other.mPosition;
    float distanceBetweenCenters = glm::length(relativePosition);
    float combinedRadius = mRadius + other.mRadius;

    // Rask radiussjekk først
    if (distanceBetweenCenters <= combinedRadius) {
        return 0.0f;  // Allerede i kontakt
    }

    glm::vec3 relativeVelocity = mVelocity - other.mVelocity;
    float positionVelocityDot = glm::dot(relativePosition, relativeVelocity);

    float discriminant = (positionVelocityDot * positionVelocityDot) -
        (glm::dot(relativeVelocity, relativeVelocity) *
            (glm::dot(relativePosition, relativePosition) - combinedRadius * combinedRadius));

    if (discriminant < 0.0f) return 1.0f;  // Ingen kollisjon mulig

    float sqrtDiscriminant = std::sqrt(discriminant);
    float t1 = (-positionVelocityDot - sqrtDiscriminant) / glm::dot(relativeVelocity, relativeVelocity);
    float t2 = (-positionVelocityDot + sqrtDiscriminant) / glm::dot(relativeVelocity, relativeVelocity);

    return (t1 >= 0.0f && t1 <= 1.0f) ? t1 :
        (t2 >= 0.0f && t2 <= 1.0f) ? t2 : 1.0f;
}



// Håndterer kollisjon mellom denne ballen og en annen ball.
// Justerer hastighet og posisjon basert på kollisjonsrespons og masse.
void Ballphys::resolveCollision(Ballphys& other) {
    glm::vec3 relativePosition = mPosition - other.mPosition;
    float distanceBetweenCenters = glm::length(relativePosition);
    float combinedRadius = mRadius + other.mRadius;

    // Bruk samme radiussjekk som i detectCollision
    if (distanceBetweenCenters > combinedRadius) {
        return;  // Ingen kollisjon
    }

    glm::vec3 collisionNormal = glm::normalize(relativePosition);
    glm::vec3 relativeVelocity = mVelocity - other.mVelocity;

    float velocityAlongNormal = glm::dot(relativeVelocity, collisionNormal);

    // Sjekk om baller beveger seg fra hverandre
    if (velocityAlongNormal > 0.0f) {
        return;
    }

    float massSum = mMass + other.mMass;
    float massDifference = mMass - other.mMass;

    glm::vec3 v1New = mVelocity -
        (2.0f * other.mMass / massSum) * velocityAlongNormal * collisionNormal;
    glm::vec3 v2New = other.mVelocity +
        (2.0f * mMass / massSum) * velocityAlongNormal * collisionNormal;

    mVelocity = v1New;
    other.mVelocity = v2New;

    // Posisjonskorreksjon
    float overlap = combinedRadius - distanceBetweenCenters;
    if (overlap > 0.0f) {
        glm::vec3 correction = 0.5f * overlap * collisionNormal;
        mPosition += correction * (other.mMass / massSum);
        other.mPosition -= correction * (mMass / massSum);
    }
}



// Håndterer interaksjon mellom ballen og underlag. 
// Beregner høyde og normalvektor til underlag ved ballens posisjon, justerer ballens hastighet og posisjon
// basert på sprett (koeffisient for restitusjon) og friksjon. Stopper ballen hvis hastigheten blir veldig lav.

void Ballphys::handleSurfaceInteraction(const Datasett& datasett) {
    
    float surfaceHeight = datasett.getHeightAt(mPosition);

    if (mPosition.y <= surfaceHeight + mRadius) {
        
        glm::vec3 normal = datasett.getSurfaceNormalAt(mPosition);

        float velocityNormal = glm::dot(mVelocity, normal);
        glm::vec3 normalComponent = velocityNormal * normal;

        glm::vec3 tangentialVelocity = mVelocity - normalComponent;

        // fra 0-1 jo høyere jo mer sprett når den treffer bakken. brukes som energi som beholdes etter sprett.
        const float coefficientOfRestitution = 0.1f;
        glm::vec3 bouncedVelocity = -normalComponent * coefficientOfRestitution;

        // hente firksjon
        float frictionCoefficient = datasett.getFrictionCoefficientAt(mPosition);

        // regne fart med friksjon om det oppstår
        glm::vec3 frictionForce = -frictionCoefficient * tangentialVelocity;
        tangentialVelocity += frictionForce * mDeltaTime;

        mVelocity = tangentialVelocity + bouncedVelocity;

        mPosition.y = surfaceHeight + mRadius;

        // om farten er liten så stopper ballen
        if (glm::length(mVelocity) < 0.2f) {
            mStopped = true;
        }
        else {
            mStopped = false; 
        }
    }
}





