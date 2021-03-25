#include "rendering_engine/VRHmd.h"

namespace rendering_engine{

VRHmd::VRHmd()
    :
    m_InitSuccess(false),
    m_nearClip(0.0f),
    m_farClip(0.0f)
{
}

VRHmd::~VRHmd(){
    if (m_InitSuccess)
        m_VRSystem->UnbindID(m_Id);

    VRSystem::ReleaseInstance();
}

bool VRHmd::Init(float nearClip, float farClip){
    m_VRSystem = VRSystem::GetInstance();
    if (m_VRSystem == nullptr){
        std::cout << "Wasn't able to initalize VR system" << std::endl;
        return false;
    }

    m_Id = m_VRSystem->GetUnboundId(RE_VR_DEVICE_CLASS_HMD);
    if (m_Id == -1){
        m_InitSuccess = false;
        std::cout << "There was no available id for HMD" << std::endl;
        return false;
    } else {
        m_VRSystem->BindID(m_Id);
        m_InitSuccess = true;
    }

    m_nearClip = nearClip;
    m_farClip = farClip;
    glm::mat4 leftEyeProj = m_VRSystem->GetProjectionMatrix(EyeType::LeftEye, m_nearClip, m_farClip);
    glm::mat4 rightEyeProj = m_VRSystem->GetProjectionMatrix(EyeType::RightEye, m_nearClip, m_farClip);
    glm::mat4 leftEyeTrans = m_VRSystem->GetEyeToHeadTransform(EyeType::LeftEye);
    glm::mat4 rightEyeTrans = m_VRSystem->GetEyeToHeadTransform(EyeType::RightEye);

    m_Camera = VRCamera(leftEyeProj, leftEyeTrans, rightEyeProj, rightEyeTrans);

    m_RecomendedRenderWidth = m_VRSystem->GetRecomendedRenderWidth();
    m_RecomendedRenderHeight = m_VRSystem->GetRecomendedRenderHeight();

    return true;
}

bool VRHmd::OnUpdate(){
    m_Transform = m_VRSystem->GetHMDTransform();
    m_Camera.SetTransform(m_Transform);
}

bool VRHmd::OnEvent(Event& e){

}

VRCamera& VRHmd::GetCamera(){
    return m_Camera;
}

const VRCamera& VRHmd::GetCamera() const{
    return m_Camera;
}

const glm::mat4 VRHmd::GetTransform() const {
    return m_Transform;
}

uint32_t VRHmd::GetId() const {
    return m_Id;
}

uint32_t VRHmd::GetRecomendedRenderSizeWidth() const {
    return m_RecomendedRenderWidth;
}

uint32_t VRHmd::GetRecomendedRenderSizeHeight() const {
    return m_RecomendedRenderHeight;
}

}