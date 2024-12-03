
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Camera.h"
#include "Shader.h"
#include "ShaderLoader.h"
#include "Datasett.h"
#include "Spheres.h"
#include "Ballphys.h"
#include "BSplineSurface.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window, std::vector<Spheres>& spheres, Camera& camera);

const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

//unsigned int VBO, VAO, EBO;

void updateBallPhysics(std::vector<Spheres>& balls, const Datasett& surface) {
    std::vector<std::pair<int, int>> collisions;

    //Detect kollisjon først
    for (size_t i = 0; i < balls.size(); ++i) {
        for (size_t j = i + 1; j < balls.size(); ++j) {
            float collisionTime = balls[i].mPhysics.detectCollision(balls[j].mPhysics);
            if (collisionTime < 1.0f) {
                collisions.emplace_back(i, j);
            }
        }
    }

    
    for (auto& collision : collisions) {
        balls[collision.first].mPhysics.resolveCollision(balls[collision.second].mPhysics);
    }

    // Update positions
    for (auto& ball : balls) {
        ball.mPhysics.updatePosition(surface);
    }
}






int main()
{

    //-----------------------------------------------------------------------------------------------//
    //-------------------------------------INITILIZE-------------------------------------------------//
    //-----------------------------------------------------------------------------------------------//
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Test Win", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glEnable(GL_DEPTH_TEST);  // Enable depth testing

    

    // build and compile our shader program
    // ------------------------------------
    Shader ourShader("Exam.vs", "Exam.fs");

    


    //-----------------------------------------------------------------------------------------------//
    //-------------------------------------Forward--Declarations-------------------------------------//
    //-----------------------------------------------------------------------------------------------//
    
    Datasett datasett;
    Camera camera;
    std::vector<Spheres> spheres;
    Spheres initialSphere(1.0f, glm::vec3(-100.0f, 0.0f, 85.0f));
    Spheres secondSphere(1.0f, glm::vec3(-85.0f, -10.0f, 80.0f));
    Ballphys ballPhysics(1.0f,1.0f,-9.81f, 0.016f);
    BSplineSurface bSplineSurface;
    
    




    //-----------------------------------------------------------------------------------------------//
    //---------------Calling all functions unless needed in render loop------------------------------//
    //-----------------------------------------------------------------------------------------------//

    datasett.loadFile("Resources/xyzfile.txt");
    datasett.triangulate();
    datasett.calculateNormals();
    std::vector<GLuint> glIndices(datasett.indices.begin(), datasett.indices.end());
    datasett.setupBuffers(datasett.points, datasett.indices);
    spheres.push_back(initialSphere);
    spheres.push_back(secondSphere);
    datasett.assignFrictionCoefficients();
    

    



    //Bruke pointcloud eller wireframe
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
 


    


    float deltaTime = 0.016f;

    //-----------------------------------------------------------------------------------------------//
    //-----------------------------------------RenderLoop--------------------------------------------//
    //-----------------------------------------------------------------------------------------------//


    while (!glfwWindowShouldClose(window))
    {
        camera.processInput(window);
        processInput(window, spheres, camera);
        updateBallPhysics(spheres, datasett);


        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view = glm::lookAt(camera.position, camera.position + camera.orientation, camera.up);
        glm::mat4 projection = glm::perspective(glm::radians(60.0f), 800.0f / 600.0f, 0.1f, 10000.0f);


        glm::mat4 model = glm::mat4(1.0f);
        

        ourShader.use();
        ourShader.setVec3("lightPos", glm::vec3(0.0f, 0.0f, -50.0f));
        ourShader.setVec3("viewPos", camera.position);
        ourShader.setVec3("lightColor", glm::vec3(1.0f, 1.0f, 1.0f)); // White light
        ourShader.setVec3("objectColor", glm::vec3(1.0f, 0.0f, 1.0f)); // Purple object color
      
        ourShader.setMat4("model", model);
        ourShader.setMat4("view", view);
        ourShader.setMat4("projection", projection);



        //Uncomment for pointcloud
        
        glPointSize(1.0f); // Adjust the point size for visibility
        glBindVertexArray(datasett.VAO);
        glDrawArrays(GL_POINTS, 0, datasett.points.size());
        glBindVertexArray(0);


        glBindVertexArray(datasett.VAO);
        glDrawElements(GL_TRIANGLES, glIndices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);



        //uncomment for Bspline flate over friksjonssonen. den er ganske liten iforhold til resten av underlaget.
        
        model = glm::mat4(1.0f);
        ourShader.setMat4("model", model);
        glUniform3fv(glGetUniformLocation(ourShader.ID, "uColor"), 1, glm::value_ptr(glm::vec3(0.0f, 1.0f, 0.0f)));  // Green color
        bSplineSurface.renderWireframe();


        for (auto& sphere : spheres) {
            sphere.renderSphere(ourShader, view, projection);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow* window, std::vector<Spheres>& spheres, Camera& camera) {
    // Check for escape key
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    static bool eKeyPressed = false;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS && !eKeyPressed) {
        // Create a new sphere at the camera's position
        spheres.push_back(Spheres(1.0f, camera.position));
        eKeyPressed = true;
    }
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_RELEASE) {
        eKeyPressed = false;
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}
