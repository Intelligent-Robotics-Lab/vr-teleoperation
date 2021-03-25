#ifndef VR_HMD_H
#define VR_HMD_H

#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "rendering_engine/VRSystem.h"
#include "rendering_engine/VRCamera.h"
#include "rendering_engine/Events/Event.h"

namespace rendering_engine{

class VRHmd {
public:
    VRHmd();
    ~VRHmd();

    bool Init(float nearClip, float farClip);

    bool OnUpdate();
    bool OnEvent(Event& e);

    VRCamera& GetCamera();
    const VRCamera& GetCamera() const;

    const glm::mat4 GetTransform() const;
    uint32_t GetId() const;
    uint32_t GetRecomendedRenderSizeWidth() const;
    uint32_t GetRecomendedRenderSizeHeight() const;

private:
    VRSystem* m_VRSystem;

    uint32_t m_Id;
    VRCamera m_Camera;
    float m_nearClip, m_farClip;
    glm::mat4 m_Transform;
    uint32_t m_RecomendedRenderWidth, m_RecomendedRenderHeight;

private:
    bool m_InitSuccess;

};

}

#endif // VR_HMD_H