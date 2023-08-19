#include "Camera.h"


void Camera::update()
{
    // update camera params
    m_front = getCameraFront();
    m_right = glm::normalize(glm::cross(m_front, worldUp));
    m_up = glm::normalize(glm::cross(m_right, m_front));

    // update translations
    const auto actualEye = m_eyeOriginal + m_eyeChange;

    // update projection matrix
    const auto ratio = static_cast<float>(m_width / m_height);
    m_projection = glm::perspective(glm::radians(m_fov), ratio, m_near, m_far);

    // update view 
    m_view = glm::lookAt(actualEye, actualEye + m_front, m_up);

    // update view projection matrix
    m_viewProjection = m_projection * m_view;
}

void Camera::init(int width, int height)
{
    m_width = width;
    m_height = height;

    m_eyeOriginal = glm::vec3(0.0f, 1.0f, 0.0f);
    m_eyeChange = glm::vec3(0.0f);

    m_yaw = -90.0f;
    m_pitch = 0.0f;

    update();
}

void Camera::onResize(int width, int height)
{
    m_width = width;
    m_height = height;
}

glm::vec3 Camera::getCameraFront() const
{
    glm::vec3 front;
    front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    front.y = sin(glm::radians(m_pitch));
    front.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));


    return glm::normalize(front);
}
