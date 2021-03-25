#include "rendering_engine/Application.h"

#include "rendering_engine/Input.h"

#include <rendering_engine/KeyCodes.h>

#include <GLFW/glfw3.h>

namespace rendering_engine {

    Application* Application::s_Instance = nullptr;

    Application::Application()
         :
        m_Running(true),
        m_WindowWidth(1280),
        m_WindowHeight(720)
    {
    }

    Application::~Application(){

    }

    bool Application::Init(){

    }

    bool Application::Run(){

    }    

    void Application::OnEvent(Event& e){

    }
    
    bool Application::Shutdown(){

    }

}