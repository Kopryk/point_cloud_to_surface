#include "Display.h"

#include "Camera.h"
#include "GraphicsApplication.h"
#include "utils/RenderUtils.h"

Display::~Display()
{
    glfwTerminate();
}

void Display::init()
{
    if (!glfwInit())
    {
        _ASSERT(false);
    }

    glfwWindowHint(GLFW_DEPTH_BITS, 32);
    m_window = glfwCreateWindow(m_windowSize.m_width, m_windowSize.m_height, "Point Cloud Modeling", nullptr, nullptr);
    if (!m_window)
    {
        _ASSERT(false);
        glfwTerminate();
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);     // vsync


    if (glewInit() != GLEW_OK)
    {
        _ASSERT(false);
        glfwTerminate();
    }

#ifdef _DEBUG
    std::cout << glGetString(GL_VERSION) << std::endl;
#endif

    // set event callbacks
    glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow*, int width, int height)
        {
            Display* display = &Display::get();
            display->setWidth(width);
            display->setHeight(height);

            //  glViewport(0, 0, width, height);
              // update camera
              //Camera* camera = &Camera::get();
             // camera->onResize(display->getRenderWindowWidth(), display->getRenderWindowHeight());
        });

    glfwSetKeyCallback(m_window, [](GLFWwindow* window, int key, int scancode, int action, int mode) {

        static Camera* camera = &Camera::get();

        if (key == GLFW_KEY_A && (action == GLFW_PRESS || action == GLFW_REPEAT))
        {
            camera->moveLeft();

        }
        else if (key == GLFW_KEY_D && (action == GLFW_PRESS || action == GLFW_REPEAT))
        {
            camera->moveRight();
        }
        else if (key == GLFW_KEY_W && (action == GLFW_PRESS || action == GLFW_REPEAT))
        {
            camera->moveForward();
        }
        else if (key == GLFW_KEY_S && (action == GLFW_PRESS || action == GLFW_REPEAT))
        {
            camera->moveBackward();
        }
        else if (key == GLFW_KEY_Q && (action == GLFW_PRESS || action == GLFW_REPEAT))
        {
            camera->moveUp();
        }
        else if (key == GLFW_KEY_E && (action == GLFW_PRESS || action == GLFW_REPEAT))
        {
            camera->moveDown();
        }

        });

    glfwSetCursorPosCallback(m_window, [](GLFWwindow* window, double xpos, double ypos)
        {
            static Display* display = &Display::get();
            display->m_lastCursorPosition = display->m_cursorPosition;
            display->m_cursorPosition.x = xpos;
            display->m_cursorPosition.y = ypos;


            if (display->m_isMouseLeftButtonPressed || display->m_isMouseRightButtonPressed)
            {
                double xDiff = display->m_cursorPosition.x - display->m_lastCursorPosition.x;
                double yDiff = display->m_cursorPosition.y - display->m_lastCursorPosition.y;
                static Camera* camera = &Camera::get();

                if (display->m_isMouseLeftButtonPressed)
                {
                    camera->moveRight(static_cast<float>(-xDiff * 0.1f));
                    camera->moveUp(static_cast<float>(yDiff * 0.1f));
                }
                else
                {
                    camera->updatePitch(static_cast<float>(yDiff * 0.1f));
                    camera->updateYaw(static_cast<float>(-xDiff * 0.1f));
                }
            }
        });

    glfwSetMouseButtonCallback(m_window, [](GLFWwindow* window, int button, int action, int mods)
        {
            static Display* display = &Display::get();
            if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
            {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                display->m_cursorPosition.x = xpos;
                display->m_cursorPosition.y = ypos;
                display->m_isMouseLeftButtonPressed = true;
            }
            else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
            {
                display->m_isMouseLeftButtonPressed = false;
            }

            if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
            {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                display->m_cursorPosition.x = xpos;
                display->m_cursorPosition.y = ypos;
                display->m_isMouseRightButtonPressed = true;
            }
            else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
            {
                display->m_isMouseRightButtonPressed = false;
            }

        });

    glfwSetScrollCallback(m_window, [](GLFWwindow* window, double xOffset, double yOffset)
        {
            static Camera* camera = &Camera::get();
            float sensitivity = static_cast<float>(yOffset);
            camera->moveForward(sensitivity);

        });

}

GLFWwindow* Display::getWindow() const
{
    return m_window;
}

int Display::getWidth() const
{
    return m_windowSize.m_width;
}

int Display::getHeight() const
{
    return m_windowSize.m_height;;
}

void Display::setWidth(int width)
{
    m_windowSize.m_width = width;
}

void Display::setHeight(int height)
{
    m_windowSize.m_height = height;
}
