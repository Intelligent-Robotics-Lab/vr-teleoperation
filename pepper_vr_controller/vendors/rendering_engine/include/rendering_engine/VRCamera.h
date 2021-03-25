#ifndef VR_CAMERA_H
#define VR_CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include "rendering_engine/VRCodes.h"

namespace rendering_engine{


class VRCamera {
public:
    VRCamera();
    VRCamera(glm::mat4 LeftEyeProjection, glm::mat4 LeftEyeToHeadTransform, glm::mat4 RightEyeProjection, glm::mat4 RightEyeToHeadTransform);
    VRCamera(glm::mat4 EyeProjection, glm::mat4 LeftEyeToHeadTransform, glm::mat4 RightEyeToHeadTransform);

    ~VRCamera();

    void SetTransform(glm::mat4 transform);

    const glm::mat4& GetProjectionMatrix(EyeType eye) const;
    const glm::mat4& GetViewMatrix(EyeType eye) const;
    const glm::mat4& GetViewProjectionMatrix(EyeType eye) const;

    const glm::vec3& GetPositionOfEye(EyeType eye) const;

private:
    struct eyeData {
        glm::mat4 projection;
        glm::mat4 view;
        glm::mat4 viewProjection;

        glm::mat4 eyeToHeadTransform;
        glm::vec3 position;
    };

    eyeData m_Eye[2];

    glm::mat4 m_Transform;

private:
    void UpdateViewMatrix();
};

}

#endif // VR_CAMERA_H