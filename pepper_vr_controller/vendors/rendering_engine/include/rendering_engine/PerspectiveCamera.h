#ifndef PERSPECIVE_CAMERA_H
#define PERSPECIVE_CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

namespace rendering_engine{

class PerspectiveCamera {
public:
    PerspectiveCamera(float FOV, float aspectRatio, float nearClip, float farClip);
    ~PerspectiveCamera();

    void SetProjection(float FOV, float aspectRatio, float nearClip, float farClip);

    float GetFOV() const;
    void SetFOV(float FOV);

    float GetAspectRatio() const;
    void SetAspectRatio(float aspectRatio);

    float GetNearClipDistance() const;
    void SetNearClipDistance(float nearClipDistance);

    float GetFarClipDistance() const;
    void SetFarClipDistance(float farClipDistance);

    const glm::vec3& GetPosition() const;
    void SetPosition(const glm::vec3& positon);

    const glm::quat& GetRotation() const;
    void SetRotation(const glm::quat& rotation);

    const glm::mat4& GetProjectionMatrix() const;
    const glm::mat4& GetViewMatrix() const;
    const glm::mat4& GetViewProjectionMatrix() const;
    
private:
    float m_FOV;
    float m_AspectRatio;
    float m_NearClipDistance;
    float m_FarClipDistance;

    glm::mat4 m_Projection;
    glm::mat4 m_View;
    glm::mat4 m_ViewProjection;

    glm::vec3 m_Position;
    glm::quat m_Rotation;

    void UpdateViewMatrix();
};

}

#endif // PERSPECIVE_CAMERA_H