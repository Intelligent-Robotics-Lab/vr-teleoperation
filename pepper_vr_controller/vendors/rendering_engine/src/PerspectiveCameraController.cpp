#include "rendering_engine/PerspectiveCameraController.h"

#include "rendering_engine/Input.h"
#include "rendering_engine/KeyCodes.h"

#include <functional>

namespace rendering_engine{

    PerspectiveCameraController::PerspectiveCameraController(float FOV, float aspectRatio, float nearClip, float farClip)
        :
        m_FOV(FOV),
        m_AspectRatio(aspectRatio),
        m_NearClip(nearClip),
        m_FarClip(farClip),
        m_Camera(FOV, aspectRatio, nearClip, farClip)
    {
        m_CameraPosition = m_Camera.GetPosition();
        m_CameraRotation = m_Camera.GetRotation();
    }

    PerspectiveCameraController::~PerspectiveCameraController(){}

    void PerspectiveCameraController::OnUpdate(Timestep ts){
        if (Input::IsKeyPressed(RE_KEY_W)){
            m_CameraPosition.z -= m_CameraTranslationSpeed*(float)ts;
        }
        else if (Input::IsKeyPressed(RE_KEY_S)){
            m_CameraPosition.z += m_CameraTranslationSpeed*(float)ts;
        }
        if (Input::IsKeyPressed(RE_KEY_A)){
            m_CameraPosition.x -= m_CameraTranslationSpeed*(float)ts;
        }
        else if (Input::IsKeyPressed(RE_KEY_D)){
            m_CameraPosition.x += m_CameraTranslationSpeed*(float)ts;
        }
        if (Input::IsKeyPressed(RE_KEY_Q)){
            m_CameraPosition.y -= m_CameraTranslationSpeed*(float)ts;
        }
        else if (Input::IsKeyPressed(RE_KEY_E)){
            m_CameraPosition.y += m_CameraTranslationSpeed*(float)ts;
        }

        m_Camera.SetPosition(m_CameraPosition);
    }

    void PerspectiveCameraController::OnEvent(Event& e){
        EventDispatcher dispatcher(e);
        dispatcher.Dispatch<WindowResizeEvent>(std::bind(&PerspectiveCameraController::OnWindowResize, this, std::placeholders::_1));
    }

    bool PerspectiveCameraController::OnWindowResize(WindowResizeEvent& e){
        m_AspectRatio = (float)e.GetWidth() / (float)e.GetHeight();
        m_Camera.SetAspectRatio(m_AspectRatio);
        return false;
    }

    PerspectiveCamera& PerspectiveCameraController::GetCamera(){
        return m_Camera;
    }

    const PerspectiveCamera& PerspectiveCameraController::GetCamera() const {
        return m_Camera;
    }
}