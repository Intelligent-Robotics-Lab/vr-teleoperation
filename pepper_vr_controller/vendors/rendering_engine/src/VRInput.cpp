#include "rendering_engine/VRInput.h"

#include <iostream>

#include "rendering_engine/Utilities.h"

namespace rendering_engine{

    VRInput* VRInput::s_Instance = nullptr;

    VRInput* VRInput::Init(){
        if (s_Instance == nullptr){
            s_Instance  = new VRInput();
        }
        return s_Instance;
    }

    VRInput::VRInput()
        :
        m_actionLeftPose(vr::k_ulInvalidActionHandle),
        m_sourceLeft(vr::k_ulInvalidInputValueHandle),
        m_actionRightPose(vr::k_ulInvalidActionHandle),
        m_sourceRight(vr::k_ulInvalidInputValueHandle),
        m_actionsetMain(vr::k_ulInvalidActionSetHandle),
        m_actionLeftTrigger(vr::k_ulInvalidActionHandle),
        m_actionRightTrigger(vr::k_ulInvalidActionHandle),
        m_actionRightGripper(vr::k_ulInvalidActionHandle),
        m_actionLeftGripper(vr::k_ulInvalidActionHandle)
    {
        vr::VRInput()->SetActionManifestPath("/home/alex/ros/src/pepper_vr_controller/vendors/rendering_engine/VRInput/actions.json");

        vr::VRInput()->GetActionHandle("/actions/main/in/Left_Pose", &m_actionLeftPose);
        vr::VRInput()->GetActionHandle("/actions/main/in/Left_Trigger", &m_actionLeftTrigger);
        vr::VRInput()->GetActionHandle("/actions/main/in/Left_Gripper", &m_actionLeftGripper);
        vr::VRInput()->GetInputSourceHandle("/user/hand/left", &m_sourceLeft);
        vr::VRInput()->GetActionHandle("/actions/main/in/Right_Pose", &m_actionRightPose);
        vr::VRInput()->GetActionHandle("/actions/main/in/Right_Trigger", &m_actionRightTrigger);
        vr::VRInput()->GetActionHandle("/actions/main/in/Right_Gripper", &m_actionRightGripper);
        vr::VRInput()->GetInputSourceHandle("/user/hand/right", &m_sourceRight);

        vr::VRInput()->GetActionSetHandle("/actions/main", &m_actionsetMain);
    }

    void VRInput::OnUpdateImpl(){
        vr::VRActiveActionSet_t actionSet = { 0 };
	    actionSet.ulActionSet = m_actionsetMain;
	    vr::VRInput()->UpdateActionState( &actionSet, sizeof(actionSet), 1 );
    }

    float VRInput::GetLeftTriggerValueImpl(){
        vr::InputAnalogActionData_t triggerData;
        if( vr::VRInput()->GetAnalogActionData(m_actionLeftTrigger, &triggerData, sizeof(triggerData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !triggerData.bActive){

        } else {
            return triggerData.x;
        }
    }

    float VRInput::GetRightTriggerValueImpl(){
        vr::InputAnalogActionData_t triggerData;
        if( vr::VRInput()->GetAnalogActionData(m_actionRightTrigger, &triggerData, sizeof(triggerData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !triggerData.bActive){

        } else {
            return triggerData.x;
        }
    }

    glm::mat4 VRInput::GetLeftHandTransformImpl(){
		vr::InputPoseActionData_t poseData;
		if ( vr::VRInput()->GetPoseActionDataForNextFrame(m_actionLeftPose, vr::TrackingUniverseStanding, &poseData, sizeof( poseData ), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None
			|| !poseData.bActive || !poseData.pose.bPoseIsValid )
		{
            std::cout << "[Input Warning] Could not obtain pose for left controller." << std::endl;
            std::cout << vr::VRInput()->GetPoseActionDataForNextFrame(m_actionLeftPose, vr::TrackingUniverseStanding, &poseData, sizeof( poseData ), vr::k_ulInvalidInputValueHandle ) << std::endl;
            std::cout << poseData.bActive << std::endl;
            std::cout << poseData.pose.bPoseIsValid << std::endl;
            return glm::mat4(0.0f);
		}
		else
		{
            return utilities::hmd34_to_mat4(poseData.pose.mDeviceToAbsoluteTracking);
		}   
    }
    
    glm::mat4 VRInput::GetRightHandTransformImpl(){
		vr::InputPoseActionData_t poseData;
		if ( vr::VRInput()->GetPoseActionDataForNextFrame(m_actionRightPose, vr::TrackingUniverseStanding, &poseData, sizeof( poseData ), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None
			|| !poseData.bActive || !poseData.pose.bPoseIsValid )
		{
            std::cout << "[Input Warning] Could not obtain pose for right controller." << std::endl;
            return glm::mat4(0.0f);
		}
		else
		{
            return utilities::hmd34_to_mat4(poseData.pose.mDeviceToAbsoluteTracking);
		}   
    }


    bool VRInput::GetLeftGripperStateImpl(){
        vr::InputDigitalActionData_t gripperData;
        if( vr::VRInput()->GetDigitalActionData(m_actionLeftGripper, &gripperData, sizeof(gripperData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !gripperData.bActive){
            return false;
        } else {
            return gripperData.bState;
        }
    }
    
    bool VRInput::GetRightGripperStateImpl(){
        vr::InputDigitalActionData_t gripperData;
        if( vr::VRInput()->GetDigitalActionData(m_actionRightGripper, &gripperData, sizeof(gripperData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !gripperData.bActive){
            return false;
        } else {
            return gripperData.bState;
        }
    }
    
    bool VRInput::GetLeftGripperPressedImpl(){
        vr::InputDigitalActionData_t gripperData;
        if( vr::VRInput()->GetDigitalActionData(m_actionLeftGripper, &gripperData, sizeof(gripperData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !gripperData.bActive){
            return false;
        } else {
            return gripperData.bState & gripperData.bChanged;
        }
    }

    bool VRInput::GetLeftGripperReleasedImpl(){
        vr::InputDigitalActionData_t gripperData;
        if( vr::VRInput()->GetDigitalActionData(m_actionLeftGripper, &gripperData, sizeof(gripperData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !gripperData.bActive){
            return false;
        } else {
            return !gripperData.bState & gripperData.bChanged;
        }
    }

    bool VRInput::GetRightGripperPressedImpl(){
        vr::InputDigitalActionData_t gripperData;
        if( vr::VRInput()->GetDigitalActionData(m_actionRightGripper, &gripperData, sizeof(gripperData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !gripperData.bActive){
            return false;
        } else {
            return gripperData.bState & gripperData.bChanged;
        }
    }

    bool VRInput::GetRightGripperReleasedImpl(){
        vr::InputDigitalActionData_t gripperData;
        if( vr::VRInput()->GetDigitalActionData(m_actionRightGripper, &gripperData, sizeof(gripperData), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None && !gripperData.bActive){
            return false;
        } else {
            return !gripperData.bState & gripperData.bChanged;
        }
    }
}