#include "rendering_engine/VRCamera.h"
#include <iostream>

namespace rendering_engine{

    VRCamera::VRCamera(){
        
    }

    VRCamera::VRCamera(glm::mat4 LeftEyeProjection, glm::mat4 LeftEyeToHeadTransform, glm::mat4 RightEyeProjection, glm::mat4 RightEyeToHeadTransform)
        :
    m_Transform(glm::mat4(1.0f))
    {
        m_Eye[LeftEye].projection = LeftEyeProjection;
        m_Eye[LeftEye].eyeToHeadTransform = LeftEyeToHeadTransform;
        m_Eye[RightEye].projection = RightEyeProjection;
        m_Eye[RightEye].eyeToHeadTransform = RightEyeToHeadTransform;

        m_Eye[LeftEye].view = m_Eye[LeftEye].eyeToHeadTransform;
        m_Eye[LeftEye].viewProjection = m_Eye[LeftEye].projection* m_Eye[LeftEye].view;

        m_Eye[RightEye].view = m_Eye[RightEye].eyeToHeadTransform;
        m_Eye[RightEye].viewProjection = m_Eye[RightEye].projection * m_Eye[RightEye].view;

        m_Eye[LeftEye].position = -m_Eye[LeftEye].view[3];
        m_Eye[RightEye].position = -m_Eye[RightEye].view[3];
    }

    VRCamera::VRCamera(glm::mat4 EyeProjection, glm::mat4 LeftEyeToHeadTransform, glm::mat4 RightEyeToHeadTransform)
        :
    m_Transform(glm::mat4(1.0f))
    {
        m_Eye[LeftEye].projection = EyeProjection;
        m_Eye[LeftEye].eyeToHeadTransform = LeftEyeToHeadTransform;
        m_Eye[RightEye].projection = EyeProjection;
        m_Eye[RightEye].eyeToHeadTransform = RightEyeToHeadTransform;

        m_Eye[LeftEye].view = glm::inverse(m_Eye[LeftEye].eyeToHeadTransform);
        m_Eye[LeftEye].viewProjection = m_Eye[LeftEye].projection* m_Eye[LeftEye].view;

        m_Eye[RightEye].view = glm::inverse(m_Eye[RightEye].eyeToHeadTransform);
        m_Eye[RightEye].viewProjection = m_Eye[RightEye].projection * m_Eye[RightEye].view;

        m_Eye[LeftEye].position = -m_Eye[LeftEye].view[3];
        m_Eye[RightEye].position = -m_Eye[RightEye].view[3];
    }

    VRCamera::~VRCamera(){
        
    }

    void VRCamera::SetTransform(glm::mat4 transform){
        m_Transform = transform;
        UpdateViewMatrix();
    }

    void VRCamera::UpdateViewMatrix(){
        for(auto& eye : m_Eye){
            eye.view = glm::inverse(m_Transform * eye.eyeToHeadTransform);
            eye.viewProjection = eye.projection * eye.view;
            eye.position = -eye.view[3];
        }
    }

    const glm::mat4& VRCamera::GetProjectionMatrix(EyeType eye) const {
        return m_Eye[eye].projection;
    }

    const glm::mat4& VRCamera::GetViewMatrix(EyeType eye) const {
        return m_Eye[eye].view;
    }

    const glm::mat4& VRCamera::GetViewProjectionMatrix(EyeType eye) const {
        return m_Eye[eye].viewProjection;
    }

    const glm::vec3& VRCamera::GetPositionOfEye(EyeType eye) const{
        return m_Eye[eye].position;
    }

}