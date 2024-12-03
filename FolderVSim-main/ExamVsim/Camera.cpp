#include "Camera.h"


Camera::Camera()
{
    // Initialize position at the origin
    position = glm::vec3(0.0f, 2.0f, 100.0f);

    // Initialize orientation to look along the negative z-axis
    orientation = glm::vec3(0.0f, 0.0f, -1.0f);

    // Initialize up direction to be the positive y-axis
    up = glm::vec3(0.0f, 1.0f, 0.0f);

    // Set window dimensions to some default values
    width = 800;
    height = 600;
    speed = 1.0f;
    sensitivity = 10.0f;

    firstClick = true;

}

void Camera::processInput(GLFWwindow* window)
{
    float cameraSpeed = 0.80f;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        position += cameraSpeed * orientation;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        position -= cameraSpeed * orientation;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        position -= glm::normalize(glm::cross(orientation, up)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        position += glm::normalize(glm::cross(orientation, up)) * cameraSpeed;

    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        position += up * speed;
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        position -= up * speed;

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS && firstClick)
    {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        firstClick = false;
    }
    else if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS && !firstClick)
    {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        firstClick = true;
    }

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        // Hides mouse cursor
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

        // Prevents camera from jumping on the first click
        if (firstClick)
        {
            glfwSetCursorPos(window, (width / 2), (height / 2));
            firstClick = false;
        }

        // Stores the coordinates of the cursor
        double mouseX;
        double mouseY;
        // Fetches the coordinates of the cursor
        glfwGetCursorPos(window, &mouseX, &mouseY);
        float rotX = sensitivity * (float)(mouseY - (height / 2)) / height;
        float rotY = sensitivity * (float)(mouseX - (width / 2)) / width;

        // Calculates upcoming vertical change in the Orientation
        glm::mat4 rotationX = glm::rotate(glm::mat4(1.0f), glm::radians(-rotX), glm::normalize(glm::cross(orientation, up)));
        glm::vec3 newOrientation = glm::vec3(rotationX * glm::vec4(orientation, 1.0f));


        float dot = glm::dot(glm::normalize(newOrientation), glm::normalize(up));
        float angle = glm::acos(dot);

        if (abs(angle - glm::radians(90.0f)) <= glm::radians(85.0f))
        {
            orientation = newOrientation;
        }

        // Rotates the Orientation left and right
        glm::mat4 rotationY = glm::rotate(glm::mat4(1.0f), glm::radians(-rotY), up);
        orientation = glm::vec3(rotationY * glm::vec4(orientation, 1.0f));

        // Sets mouse cursor to the middle of the screen so that it doesn't end up roaming around
        glfwSetCursorPos(window, (width / 2), (height / 2));
    }
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
    {
        // Unhides cursor since camera is not looking around anymore
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        // Makes sure the next time the camera looks around it doesn't jump
        firstClick = true;
    }
}
