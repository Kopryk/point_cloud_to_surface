#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>


class Camera
{
public:
    static Camera& get()
    {
        static Camera instance{};
        return instance;
    }

    ~Camera() = default;
    Camera(const Camera&) = delete;
    Camera(Camera&&) = delete;
    Camera& operator=(const Camera&) = delete;
    Camera& operator=(Camera&&) = delete;

    void update();
    void init(int width, int height);

    [[nodiscard]] glm::mat4 getViewProjection() const { return m_viewProjection; }

    void onResize(int width, int height);

    float& getPitchRef() { return m_pitch; }
    float& getYawRef() { return m_yaw; }
    float& getEyeChangeXRef() { return m_eyeChange.x; };
    float& getEyeChangeZRef() { return m_eyeChange.z; };
    float& getEyeChangeYRef() { return m_eyeChange.y; };
    float& getFovRef() { return m_fov; };
    float& getNearRef() { return m_near; };
    float& getFarRef() { return m_far; };
    float& getCameraSpeedRef() { return m_cameraSpeed; }
    float& getScrollSpeedRef() { return m_scrollSpeed; }

    void moveLeft(float sensitivity = 1.0f) { m_eyeChange -= glm::normalize(glm::cross(m_front, m_up)) * m_cameraSpeed * sensitivity; }
    void moveRight(float sensitivity = 1.0f) { m_eyeChange += glm::normalize(glm::cross(m_front, m_up)) * m_cameraSpeed * sensitivity; }
    void moveUp(float sensitivity = 1.0f) { m_eyeChange += m_up * m_cameraSpeed * sensitivity; }
    void moveDown(float sensitivity = 1.0f) { m_eyeChange -= m_up * m_cameraSpeed * sensitivity; }
    void moveForward() { m_eyeChange += m_front * m_cameraSpeed; }
    void moveForward(float sensitivity) { m_eyeChange += m_front * m_cameraSpeed * sensitivity * m_scrollSpeed; }

    void moveBackward() { m_eyeChange -= m_front * m_cameraSpeed; }

    void updatePitch(float pitchChange) { m_pitch += pitchChange; }
    void updateYaw(float yawChange) { m_yaw += yawChange; }

    void lookAtPoint(float x, float y, float z);

private:

    glm::mat4 m_view = glm::mat4(1.0f);
    glm::mat4 m_projection = glm::mat4(1.0f);
    glm::mat4 m_viewProjection = glm::mat4(1.0f);

    glm::vec3 m_eyeOriginal = glm::vec3(0.0f);
    glm::vec3 m_eyeChange = glm::vec3(0.0f);
    glm::vec3 m_right = glm::vec3(0.0f);
    glm::vec3 m_up = glm::vec3(0.0f);
    glm::vec3 m_front = glm::vec3(0.0f);

    glm::vec3 worldUp = glm::vec3(0.0f, 1.0f, 0.0f);

    float m_pitch = 0.0f;
    float m_yaw = 0.0f;
    float m_fov = 45.0f;
    float m_near = 0.1f;
    float m_far = 100.0f;
    int m_width = 0;
    int m_height = 0;
    float m_cameraSpeed = 0.05f;
    float m_scrollSpeed = 3.0f;

    Camera() = default;
    [[nodiscard]] glm::vec3 getCameraFront() const;

};
