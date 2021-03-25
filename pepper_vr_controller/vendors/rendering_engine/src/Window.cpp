#include "rendering_engine/Window.h"

#include "rendering_engine/Events/ApplicationEvent.h"
#include "rendering_engine/Events/KeyEvent.h"
#include "rendering_engine/Events/MouseEvent.h"

namespace rendering_engine{

Window::Window(){
    Init("rendering_engine", 1280, 720, true);
}


Window::Window(const std::string& title, unsigned int width, unsigned int height, bool resizable){
    Init(title, width, height, resizable);
}

Window::~Window(){
    Shutdown();
}

void Window::OnUpdate(){
    glfwSwapBuffers(m_Window);
    glfwPollEvents();
}

bool Window::IsVSync() const { return m_Data.vSync; }
bool Window::IsResizable() const { return m_Data.resizable; }
unsigned int Window::GetWidth() const { return m_Data.width; }
unsigned int Window::GetHeight() const { return m_Data.height; }

void Window::SetVSync(bool enabled){
    if(enabled)
        glfwSwapInterval(1);
    else
        glfwSwapInterval(0);

    m_Data.vSync = enabled;
}

bool Window::Init(const std::string& Title, unsigned int Width, unsigned int Height, bool Resizable){
    m_Data.title = Title;
    m_Data.width = Width;
    m_Data.height = Height;
    m_Data.resizable = Resizable;

    int success = glfwInit();
    if(success != GLFW_TRUE){
        printf("GLFW initialization failed");
        glfwTerminate();
        return false;
    }

    if (!m_Data.resizable) glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_Window = glfwCreateWindow(m_Data.width, m_Data.height, m_Data.title.c_str(), nullptr, nullptr);
    if(!m_Window){
        printf("GLFW window creation failed");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_Window);

    glfwSetWindowUserPointer(m_Window, &m_Data);

    SetVSync(true);
    m_Data.vSync = true;

    // Set GLFW callbacks
    glfwSetWindowSizeCallback(m_Window, [](GLFWwindow* window, int width, int height){
        WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);
        data.width = width;
        data.height = height;

        WindowResizeEvent event(width, height);
        data.EventCallback(event);
    });

    glfwSetWindowCloseCallback(m_Window, [](GLFWwindow* window){
        WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);
        WindowCloseEvent event;
        data.EventCallback(event);
    });

    glfwSetKeyCallback(m_Window, [](GLFWwindow* window, int key, int scancode, int action, int mods){
        WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

        switch (action)
        {
            case GLFW_PRESS:
            {
                KeyPressedEvent event(static_cast<KeyCode>(key), 0);
                data.EventCallback(event);
                break;
            }
            case GLFW_RELEASE:
            {
                KeyReleasedEvent event(static_cast<KeyCode>(key));
                data.EventCallback(event);
                break;
            }        
            case GLFW_REPEAT:
            {
                KeyPressedEvent event(static_cast<KeyCode>(key), 1);
                data.EventCallback(event);
                break;
            }
        }
    });

    glfwSetMouseButtonCallback(m_Window, [](GLFWwindow* window, int button, int action, int mods){
        WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

        switch (action)
        {
            case GLFW_PRESS:
            {
                MouseButtonPressedEvent event(static_cast<MouseCode>(button));
                data.EventCallback(event);
                break;
            }
            case GLFW_RELEASE:
            {
                MouseButtonReleasedEvent event(static_cast<MouseCode>(button));
                data.EventCallback(event);
                break;
            }        
        }
    });

    glfwSetScrollCallback(m_Window, [](GLFWwindow* window, double xOffset, double yOffset){
        WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

        MouseScrolledEvent event((float)xOffset, (float)yOffset);
        data.EventCallback(event);
    });

    glfwSetCursorPosCallback(m_Window, [](GLFWwindow* window, double xPos, double yPos){
        WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

        MouseMovedEvent event((float)xPos, (float)yPos);
        data.EventCallback(event);
    });

    return true;
}

void Window::Shutdown(){
    glfwDestroyWindow(m_Window);
}

}
