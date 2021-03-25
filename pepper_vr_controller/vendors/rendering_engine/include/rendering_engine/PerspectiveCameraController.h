#ifndef PERSPECTIVE_CAMERA_CONTROLLER_H
#define PERSPECTIVE_CAMERA_CONTROLLER_H

#include "rendering_engine/PerspectiveCamera.h"

#include "rendering_engine/Timestep.h"

#include "rendering_engine/Events/ApplicationEvent.h"

namespace rendering_engine{

class PerspectiveCameraController{
public:
    PerspectiveCameraController(float FOV, float aspectRatio, float nearClip, float farClip);
    ~PerspectiveCameraController();

    void OnUpdate(Timestep ts);
    void OnEvent(Event& e);

    PerspectiveCamera& GetCamera();
    const PerspectiveCamera& GetCamera() const;
private:
    bool OnWindowResize(WindowResizeEvent& e);
private:
    float m_FOV;
    float m_AspectRatio;
    float m_NearClip;
    float m_FarClip;

    PerspectiveCamera m_Camera;

    glm::vec3 m_CameraPosition;
    glm::quat m_CameraRotation;

    float m_CameraTranslationSpeed = 5.0f, m_CameraRotationSpeed = 45.0f;
};

}

#endif // PERSPECTIVE_CAMERA_CONTROLLER_H