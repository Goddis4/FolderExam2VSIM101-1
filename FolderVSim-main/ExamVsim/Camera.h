#pragma once
#include <glad/glad.h>
#include<GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Camera
{
public:

    Camera();
    void processInput(GLFWwindow* window);

    glm::vec3 position;
    glm::vec3 orientation;
    glm::vec3 up;

    float speed;
    float sensitivity;
    int width;
    int height;
    bool firstClick;

};


