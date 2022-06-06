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
            
            inline static bool GetForwardDpadState() {return s_Instance->GetForwardDpadStateImpl(); }
          //  inline static bool GetForwardDpadPress() {return s_Instance->GetForwardDpadPressImpl(); }
          //  inline static bool GetForwardDpadRelease() {return s_Instance->GetForwardDpadReleaseImpl(); }
            
            inline static bool GetBackwardDpadState() {return s_Instance->GetBackwardDpadStateImpl(); }
           // inline static bool GetBackwardDpadPress() {return s_Instance->GetBackwardDpadPressImpl(); }
           // inline static bool GetBackwardDpadRelease() {return s_Instance->GetBackwardDpadReleaseImpl(); }

            inline static bool GetRightDpadState() {return s_Instance->GetRightDpadStateImpl(); }
          //  inline static bool GetRightDpadPress() {return s_Instance->GetRightDpadPressImpl(); }
          //  inline static bool GetRightDpadRelease() {return s_Instance->GetRightDpadReleaseImpl(); }

            inline static bool GetLeftDpadState() {return s_Instance->GetLeftDpadStateImpl(); }
          //  inline static bool GetLeftDpadPress() {return s_Instance->GetLeftDpadPressImpl(); }
          //  inline static bool GetLeftDpadRelease() {return s_Instance->GetLeftDpadReleaseImpl(); }


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

            //// New code for dpad movement 5/2022
            bool GetForwardDpadStateImpl();
          // bool GetForwardDpadPressImpl();
           // bool GetForwardDpadReleaseImpl();

            bool GetBackwardDpadStateImpl();
          //  bool GetBackwardDpadPressImpl();
           // bool GetBackwardDpadReleaseImpl();

            bool GetLeftDpadStateImpl();
           // bool GetLeftDpadPressImpl();
           // bool GetLeftDpadReleaseImpl();

            bool GetRightDpadStateImpl();
           // bool GetRightDpadPressImpl();
           // bool GetRightDpadReleaseImpl();




        private:
            vr::VRActionHandle_t m_actionRightPose;
            vr::VRActionHandle_t m_actionRightTrigger;
            vr::VRActionHandle_t m_actionRightGripper;
            vr::VRInputValueHandle_t m_sourceRight;
            vr::VRActionHandle_t m_actionLeftPose;
            vr::VRActionHandle_t m_actionLeftTrigger;
            vr::VRActionHandle_t m_actionLeftGripper;
            vr::VRInputValueHandle_t m_sourceLeft;
            
            //new code for dpad movement 5/22
            vr::VRActionHandle_t m_actionDpadMoveForward;
            vr::VRActionHandle_t m_actionDpadMoveBackward;
            vr::VRActionHandle_t m_actionDpadTurnRight;
            vr::VRActionHandle_t m_actionDpadTurnLeft;
            //

            vr::VRActionSetHandle_t m_actionsetMain;

        private:
            static VRInput* s_Instance;
            VRInput();
            ~VRInput();

    };

}

#endif // VR_INTPUT_H