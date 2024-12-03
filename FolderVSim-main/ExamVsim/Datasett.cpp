#include "Datasett.h"

#include <delaunator.hpp>


// Laster ned data fra xyzfil. emplace_back xZy for datafilen bruker z som hoyde. så får z fra høydedata til å bli Y akse i opengl
void Datasett::loadFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::string line;
    int numPoints;
    std::getline(file, line);
    std::istringstream(line) >> numPoints;

    // Reset points and prepare for new data
    points.clear();

    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        float x, y, z;
        ss >> x >> y >> z;

        points.emplace_back(x, z, y);

        minX = std::min(minX, x);
        minY = std::min(minY, z);
        minZ = std::min(minZ, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, z);
        maxZ = std::max(maxZ, y);
    }
    file.close();

    // sentrer punkt på origin
    glm::vec3 center((minX + maxX) / 2.0f, (minY + maxY) / 2.0f, (minZ + maxZ) / 2.0f);
    for (auto& p : points) {
        p -= center;
    }

    std::cout << "Loaded " << points.size() << " points successfully." << std::endl;
}




//Triangulering med delaunay ved bruk av library
void Datasett::triangulate() {
    if (points.empty()) {
        std::cerr << "No points to triangulate." << std::endl;
        return;
    }

    
    std::vector<double> coords;
    for (const auto& p : points) {
        coords.push_back(p.x); 
        coords.push_back(p.z); 
    }

    
    // Triangulerings caller
    delaunator::Delaunator d(coords);

    indices = d.triangles;


    std::cout << "Triangulation completed with " << indices.size() / 3 << " triangles." << std::endl;
}



//kalkulere normaler slik at de kan brukes til phong shading 
void Datasett::calculateNormals() {
    // setter alle normaler til 0
    normals.resize(points.size(), glm::vec3(0.0f));

    // for å så gå over alle trekantene fra trianguleringen
    for (size_t i = 0; i < indices.size(); i += 3) {
        size_t i0 = indices[i];
        size_t i1 = indices[i + 1];
        size_t i2 = indices[i + 2];

        glm::vec3 v0 = points[i0];
        glm::vec3 v1 = points[i1];
        glm::vec3 v2 = points[i2];

        // triangel normaler
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));

        // normaler per vertex
        normals[i0] += normal;
        normals[i1] += normal;
        normals[i2] += normal;
    }

    // Normalisere normalene
    for (auto& normal : normals) {
        normal = glm::normalize(normal);
    }

    std::cout << "Normals calculated" << std::endl;
}



//kalkulere barysentriske koordinater
glm::vec3 Datasett::computeBarycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const {
    glm::vec3 v0 = b - a;
    glm::vec3 v1 = c - a;
    glm::vec3 v2 = p - a;

    
    glm::vec2 p2d = glm::vec2(p.x, p.z);
    glm::vec2 a2d = glm::vec2(a.x, a.z);
    glm::vec2 b2d = glm::vec2(b.x, b.z);
    glm::vec2 c2d = glm::vec2(c.x, c.z);

    glm::vec2 v0_2d = b2d - a2d;
    glm::vec2 v1_2d = c2d - a2d;
    glm::vec2 v2_2d = p2d - a2d;

    float d00 = glm::dot(v0_2d, v0_2d);
    float d01 = glm::dot(v0_2d, v1_2d);
    float d11 = glm::dot(v1_2d, v1_2d);
    float d20 = glm::dot(v2_2d, v0_2d);
    float d21 = glm::dot(v2_2d, v1_2d);

    float denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-6) {
        return glm::vec3(-1.0f); 
    }

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    
    const float epsilon = 1e-6f;
    if (u >= -epsilon && v >= -epsilon && w >= -epsilon) {
        
        u = std::max(0.0f, u);
        v = std::max(0.0f, v);
        w = std::max(0.0f, w);

        //u + v + w = 1
        float sum = u + v + w;
        return glm::vec3(u / sum, v / sum, w / sum);
    }

    return glm::vec3(-1.0f);
}




//kalkulere normaler for surface med barycentriske koordinater som skal brukes for ball kollisjon.
glm::vec3 Datasett::getSurfaceNormalAt(const glm::vec3& position) const {
    glm::vec3 interpolatedNormal(0.0f);
    float closestDistance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < indices.size(); i += 3) {
        size_t i0 = indices[i];
        size_t i1 = indices[i + 1];
        size_t i2 = indices[i + 2];

        glm::vec3 v0 = points[i0];
        glm::vec3 v1 = points[i1];
        glm::vec3 v2 = points[i2];

        //sjekke om posisjonen er inni et triangel
        glm::vec3 baryCoords = computeBarycentric(position, v0, v1, v2);

        if (baryCoords.x >= 0 && baryCoords.y >= 0 && baryCoords.z >= 0) {
            // interpolere med hjelp av barysentriske koordinater
            interpolatedNormal = glm::normalize(
                baryCoords.x * normals[i0] +
                baryCoords.y * normals[i1] +
                baryCoords.z * normals[i2]
            );
            return interpolatedNormal; //return om du er inne i et triangel
        }

        //om den ikke er i et triangel så må jeg kalkulere nermeste punkt
        glm::vec3 closestPoint = calculateClosestPointOnTriangle(position, v0, v1, v2);
        float distance = glm::length(closestPoint - position);

        if (distance < closestDistance) {
            closestDistance = distance;

            //interpolerer normaler
            glm::vec3 closestBaryCoords = computeBarycentric(closestPoint, v0, v1, v2);
            interpolatedNormal = glm::normalize(
                closestBaryCoords.x * normals[i0] +
                closestBaryCoords.y * normals[i1] +
                closestBaryCoords.z * normals[i2]
            );
        }
    }

    return interpolatedNormal;
}




//hente høyde med hjelp av barysentriske koordinater 
float Datasett::getHeightAt(const glm::vec3& position) const {
    float closestHeight = std::numeric_limits<float>::lowest();
    float closestDistance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < indices.size(); i += 3) {
        size_t i0 = indices[i];
        size_t i1 = indices[i + 1];
        size_t i2 = indices[i + 2];

        glm::vec3 v0 = points[i0];
        glm::vec3 v1 = points[i1];
        glm::vec3 v2 = points[i2];

        
        glm::vec3 baryCoords = computeBarycentric(position, v0, v1, v2);

        if (baryCoords.x >= 0 && baryCoords.y >= 0 && baryCoords.z >= 0) {
            //interpolerer høyde 
            float height = baryCoords.x * v0.y + baryCoords.y * v1.y + baryCoords.z * v2.y;
            return height;
        }

        
        glm::vec3 closestPoint = calculateClosestPointOnTriangle(position, v0, v1, v2);
        float distance = glm::length(closestPoint - position);

        if (distance < closestDistance) {
            closestDistance = distance;
            closestHeight = calculateHeightAtPoint(closestPoint, v0, v1, v2);
        }
    }

    
    return closestHeight;
}




// kalkulere nermeste punkt på triangel
glm::vec3 Datasett::calculateClosestPointOnTriangle(const glm::vec3& p,
    const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const {
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ap = p - a;

    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);
    float d3 = glm::dot(ab, ab);
    float d4 = glm::dot(ac, ac);
    float d5 = glm::dot(ab, ac);

    float denom = d3 * d4 - d5 * d5;
    float vc = (d1 * d4 - d2 * d5) / denom;
    float vb = (d2 * d3 - d1 * d5) / denom;
    float va = 1.0f - vc - vb;

    // Clamp til triangel
    va = std::max(0.0f, std::min(1.0f, va));
    vb = std::max(0.0f, std::min(1.0f, vb));
    vc = std::max(0.0f, std::min(1.0f, vc));

    return a * va + b * vb + c * vc;
}

// hjelpefuncsjon som kalkulerer høyde på spesifike punkt
float Datasett::calculateHeightAtPoint(const glm::vec3& point,
    const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const {
    glm::vec3 baryCoords = computeBarycentric(point, a, b, c);
    return baryCoords.x * a.y + baryCoords.y * b.y + baryCoords.z * c.y;
}



//lage en sone som inneholder friksjon
bool Datasett::isInFrictionZone(size_t triangleIndex) const {
    const glm::vec3& v0 = points[indices[triangleIndex * 3]];
    const glm::vec3& v1 = points[indices[triangleIndex * 3 + 1]];
    const glm::vec3& v2 = points[indices[triangleIndex * 3 + 2]];

    //sette denne sonen rundt origin
    glm::vec3 centroid = (v0 + v1 + v2) / 3.0f;

    //radius til sonen. jo høyere, jo større radius
    float frictionZoneRadius = 35.0f;

    //sjekke om sentrert er i friksjon sonen
    return glm::length(glm::vec2(centroid.x, centroid.z)) <= frictionZoneRadius;
}




// Henter friksjonskoeffisienten for trekanten som inneholder posisjonen.
// Hvis posisjonen ikke ligger i noen trekant, returneres 0.0f (friksjonsfritt)
float Datasett::getFrictionCoefficientAt(const glm::vec3& position) const {
    for (size_t i = 0; i < indices.size(); i += 3) {
        size_t i0 = indices[i];
        size_t i1 = indices[i + 1];
        size_t i2 = indices[i + 2];

        glm::vec3 v0 = points[i0];
        glm::vec3 v1 = points[i1];
        glm::vec3 v2 = points[i2];

        // sjekke om posisjonen er inne i et trekant
        glm::vec3 baryCoords = computeBarycentric(position, v0, v1, v2);

        if (baryCoords.x >= 0 && baryCoords.y >= 0 && baryCoords.z >= 0) {
            
            return frictionCoefficients[i / 3];
        }
    }
    return 0.0f;
}


// Tildeler friksjonskoeffisienter til hver trekant i datasettet.
// Trekantenes friksjon bestemmes basert på om de er i en friksjonssone
void Datasett::assignFrictionCoefficients() {
    frictionCoefficients.resize(indices.size() / 3);

    for (size_t i = 0; i < indices.size() / 3; ++i) {
        
        if (isInFrictionZone(i)) {
            frictionCoefficients[i] = 0.5f; // høy friksjon
        }
        else {
            frictionCoefficients[i] = 0.0f; // ingen friksjon
        }
    }
}




//Buffers for datasett
void Datasett::setupBuffers(const std::vector<glm::vec3>& points, const std::vector<size_t>& triangles) {
    
    std::vector<GLuint> glIndices(triangles.begin(), triangles.end());

    
    std::vector<float> frictionFlags(points.size(), 0.0f);
    for (size_t i = 0; i < triangles.size(); i += 3) {
        bool isFriction = isInFrictionZone(i / 3); //sjekke om den er i friksjonsonen
        for (int j = 0; j < 3; ++j) {
            frictionFlags[triangles[i + j]] = isFriction ? 1.0f : 0.0f;
        }
    }

    // Create and bind VAO
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    // Vertex positions
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    // Normals
    glGenBuffers(1, &normalVBO);
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(1);

    // Friction flags
    GLuint frictionVBO;
    glGenBuffers(1, &frictionVBO);
    glBindBuffer(GL_ARRAY_BUFFER, frictionVBO);
    glBufferData(GL_ARRAY_BUFFER, frictionFlags.size() * sizeof(float), frictionFlags.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void*)0);
    glEnableVertexAttribArray(2);

    // Triangle indices
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, glIndices.size() * sizeof(GLuint), glIndices.data(), GL_STATIC_DRAW);

    // Unbind VAO and buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}


