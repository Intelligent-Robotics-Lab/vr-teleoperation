#ifndef VR_INPUT_H
#define VR_INPUT_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <openvr.h>

#include "rendering_engine/VRCodes.h"

namespace rendering_engine{

    class VRInput{
        public:
            static VRInput* Init();

            inline static void OnUpdate() { s_Instance->OnUpdateImpl(); }

            inline static glm::mat4 GetLeftHandTransform() { return s_Instance->GetLeftHandTransformImpl(); }
            inline static glm::mat4 GetRightHandTransform() { return s_Instance->GetRightHandTransformImpl(); }

            inline static float GetLeftTriggerValue() { return s_Instance->GetLeftTriggerValueImpl(); }
            inline static float GetRightTriggerValue() { return s_Instance->GetRightTriggerValueImpl(); }

            inline static bool GetLeftGripperState() { return s_Instance->GetLeftGripperStateImpl(); }
            inline static bool GetLeftGripperPressed() { return s_Instance->GetLeftGripperPressedImpl(); }
            inline static bool GetLeftGripperReleased() { return s_Instance->GetLeftGripperReleasedImpl(); }

            inline static bool GetRightGripperState() { return s_Instance->GetRightGripperStateImpl(); }
            inline static bool GetRightGripperPressed() { return s_Instance->GetRightGripperPressedImpl(); }
            inline static bool GetRightGripperReleased() { return s_Instance->GetRightGripperReleasedImpl(); }


        private:
            void OnUpdateImpl();

            glm::mat4 GetLeftHandTransformImpl();
            glm::mat4 GetRightHandTransformImpl();

            float GetLeftTriggerValueImpl();
            float GetRightTriggerValueImpl();

            bool GetLeftGripperStateImpl();
            bool GetLeftGripperPressedImpl();
            bool GetLeftGripperReleasedImpl();

            bool GetRightGripperStateImpl();
            bool GetRightGripperPressedImpl();
            bool GetRightGripperReleasedImpl();

        private:
            vr::VRActionHandle_t m_actionRightPose;
            vr::VRActionHandle_t m_actionRightTrigger;
            vr::VRActionHandle_t m_actionRightGripper;
            vr::VRInputValueHandle_t m_sourceRight;
            vr::VRActionHandle_t m_actionLeftPose;
            vr::VRActionHandle_t m_actionLeftTrigger;
            vr::VRActionHandle_t m_actionLeftGripper;
            vr::VRInputValueHandle_t m_sourceLeft;
            
            vr::VRActionSetHandle_t m_actionsetMain;

        private:
            static VRInput* s_Instance;
            VRInput();
            ~VRInput();

    };

}

#endif // VR_INTPUT_H