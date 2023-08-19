#pragma once

// clang-format off
#include <gl/glew.h>
#include <GLFW/glfw3.h>
// clang-format on

class Display
{
public:

    static Display& get()
    {
        static Display instance;
        return instance;
    }

    ~Display();
    Display(const Display&) = delete;
    Display(Display&&) = delete;
    Display& operator = (const Display&) = delete;
    Display& operator = (Display&&) = delete;

    [[nodiscard]] GLFWwindow* getWindow() const;
    [[nodiscard]] int getWidth() const;
    [[nodiscard]] int getHeight() const;
    void setWidth(int width);
    void setHeight(int height);
    void init();

    int getRenderWindowWidth() const { return m_renderWindowSize.m_width; }
    int getRenderWindowHeight() const { return m_renderWindowSize.m_height; }


private:

    Display() = default;

    GLFWwindow* m_window = nullptr;

    struct
    {
        int m_width = 1320;
        int m_height = 1000;
    }m_windowSize;

    struct
    {
        int m_width = 900;
        int m_height = 900;
    }m_renderWindowSize;

    struct
    {
        double x, y;
    } m_cursorPosition, m_lastCursorPosition;

    bool m_isMouseLeftButtonPressed = false;
    bool m_isMouseRightButtonPressed = false;

};
