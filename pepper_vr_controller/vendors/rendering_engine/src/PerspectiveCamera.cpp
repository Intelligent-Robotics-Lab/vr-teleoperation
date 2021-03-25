#include "rendering_engine/PerspectiveCamera.h"

namespace rendering_engine{

    PerspectiveCamera::PerspectiveCamera(float fov, float aspectRatio, float nearClip, float farClip)
            :
        m_FOV(fov),
        m_AspectRatio(aspectRatio),
        m_NearClipDistance(nearClip),
        m_FarClipDistance(farClip),
        m_View(glm::mat4(1.0f)),
        m_Projection(glm::perspective(fov, aspectRatio, nearClip, farClip)),
        m_Position(glm::vec3(0.0f, 0.0f, 0.0f)),
        m_Rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
    {  
        m_ViewProjection = m_Projection * m_View;
    }

    PerspectiveCamera::~PerspectiveCamera(){  }

    void PerspectiveCamera::SetProjection(float FOV, float aspectRatio, float nearClip, float farClip){
        m_FOV = FOV;
        m_AspectRatio = aspectRatio;
        m_NearClipDistance = nearClip;
        m_FarClipDistance = farClip;

        m_Projection = glm::perspective(m_FOV, m_AspectRatio, m_NearClipDistance, m_FarClipDistance);
    }

    float PerspectiveCamera::GetFOV() const{
        return m_FOV;
    }

    void PerspectiveCamera::SetFOV(float FOV){
        m_FOV = FOV;
        m_Projection = glm::perspective(m_FOV, m_AspectRatio, m_NearClipDistance, m_FarClipDistance);
        m_ViewProjection = m_Projection * m_View;
    }

    float PerspectiveCamera::GetAspectRatio() const{
        return m_AspectRatio;
    }

    void PerspectiveCamera::SetAspectRatio(float aspectRatio){
        m_AspectRatio = aspectRatio;
        m_Projection = glm::perspective(m_FOV, m_AspectRatio, m_NearClipDistance, m_FarClipDistance);
        m_ViewProjection = m_Projection * m_View;
    }

    float PerspectiveCamera::GetNearClipDistance() const{
        return m_NearClipDistance;
    }

    void PerspectiveCamera::SetNearClipDistance(float nearClipDistance){
        m_NearClipDistance = nearClipDistance;
        m_Projection = glm::perspective(m_FOV, m_AspectRatio, m_NearClipDistance, m_FarClipDistance);
        m_ViewProjection = m_Projection * m_View;
    }

    float PerspectiveCamera::GetFarClipDistance() const{
        return m_FarClipDistance;
    }

    void PerspectiveCamera::SetFarClipDistance(float farClipDistance){
        m_FarClipDistance = farClipDistance;
        m_Projection = glm::perspective(m_FOV, m_AspectRatio, m_NearClipDistance, m_FarClipDistance);
        m_ViewProjection = m_Projection * m_View;
    }

    const glm::vec3& PerspectiveCamera::GetPosition() const{
        return m_Position;
    }

    void PerspectiveCamera::SetPosition(const glm::vec3& positon){
        m_Position = positon;
        UpdateViewMatrix();
    }

    const glm::quat& PerspectiveCamera::GetRotation() const{
        return m_Rotation;
    }

    void PerspectiveCamera::SetRotation(const glm::quat& rotation){
        m_Rotation = rotation;
        UpdateViewMatrix();
    }

    const glm::mat4& PerspectiveCamera::GetProjectionMatrix() const{
        return m_Projection;
    }

    const glm::mat4& PerspectiveCamera::GetViewMatrix() const{
        return m_View;
    }

    const glm::mat4& PerspectiveCamera::GetViewProjectionMatrix() const{
        return m_ViewProjection;
    }

    void PerspectiveCamera::UpdateViewMatrix(){
        glm::mat4 transform = glm::translate(glm::mat4(1.0f), m_Position);
        transform = transform * glm::toMat4(m_Rotation);
        m_View = glm::inverse(transform);

        m_ViewProjection = m_Projection * m_View;
    }
}