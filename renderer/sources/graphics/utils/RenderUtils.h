#pragma once
#include <GLFW/glfw3.h>
#include <iostream>

namespace RenderErrors  // TODO
{
    static void clearError() {

        while (glGetError() != GL_NO_ERROR);
    }

    static void checkError()
    {
        if (GLenum error = glGetError())
        {
            std::cout << "[GL ERROR: ]: " << error << "\n";
            _ASSERT(false);
        }
    }
}

