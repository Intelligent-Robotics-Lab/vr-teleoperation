#ifndef APPLICATION_H
#define APPLICATION_H

#include <memory>

#include "rendering_engine/Events/Event.h"
#include "rendering_engine/Window.h"

namespace rendering_engine{

class Application{ 

    public:
        Application();
        virtual ~Application();

        virtual bool Init();
        virtual bool Run();
        virtual bool Shutdown();

        virtual void OnEvent(Event& e);

        Window& GetWindow() {return *m_MainWindow;}
        static Application& Get() {return *s_Instance;}
        
    protected:
        bool m_Running;
        std::unique_ptr<Window> m_MainWindow;
        unsigned int m_WindowWidth, m_WindowHeight;

    protected:
        static Application* s_Instance;

};
} // namespace rendering_engine

#endif // APPLICATION_H