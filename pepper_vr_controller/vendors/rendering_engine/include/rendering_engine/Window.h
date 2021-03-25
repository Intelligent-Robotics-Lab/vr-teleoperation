#ifndef WINDOW_H
#define WINDOW_H

#include <string>

#include <GLFW/glfw3.h>
#include <functional>
#include "rendering_engine/Events/Event.h"

namespace rendering_engine{

class Window
{
public:
    Window();
    Window(const std::string& title, unsigned int width, unsigned int height, bool resizable);
    ~Window();

    using EventCallbackFn = std::function<void(Event&)>;

    void OnUpdate();
    unsigned int GetWidth() const;
    unsigned int GetHeight() const;

    void SetEventCallback(const EventCallbackFn& callback) {m_Data.EventCallback = callback;}

    void* GetWindowPointer() const {return m_Window;}

    void SetVSync(bool enabled);
    bool IsVSync() const;

    bool IsResizable() const;
    
private:
    struct WindowData{
        std::string title;
        unsigned int width;
        unsigned int height;
        bool vSync;
        bool resizable;

        EventCallbackFn EventCallback;
    };

    WindowData m_Data;

    GLFWwindow* m_Window;

    bool Init(const std::string& Title, unsigned int Width, unsigned int Height, bool Resizable);
    void Shutdown();
};

}

#endif // WINDOW_H