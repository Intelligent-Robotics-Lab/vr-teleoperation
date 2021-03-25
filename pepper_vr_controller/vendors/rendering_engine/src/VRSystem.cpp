#include "rendering_engine/VRSystem.h"

#include "rendering_engine/Events/VREvent.h"

#include "rendering_engine/Utilities.h"

namespace rendering_engine{

    VRSystem* VRSystem::s_Instance = nullptr;
    int VRSystem::s_Counter = 0;

    VRSystem* VRSystem::GetInstance(){
        if (s_Instance == nullptr){

            s_Instance = new VRSystem();
            if (!s_Instance->Init()){
                delete s_Instance;
                s_Instance = nullptr;
            }
        }
        AddRef();
        return s_Instance;
    }

    void VRSystem::ReleaseInstance()
	{
		ReleaseRef();	

		if((0 == s_Counter)&& (s_Instance != nullptr))
		{
			delete s_Instance;
			s_Instance = nullptr;						
		}
	}

    void VRSystem::AddRef(){ ++s_Counter; }
    void VRSystem::ReleaseRef(){ --s_Counter; }

    VRSystem::VRSystem(){
    }

    VRSystem::~VRSystem(){
        Shutdown();
    }

    bool VRSystem::Init(){

        // Initializing OpenVR with SteamVR
        vr::HmdError err;
        m_VRSystem = vr::VR_Init(&err, vr::EVRApplicationType::VRApplication_Scene);
        if(m_VRSystem == nullptr){
            std::cout << "Error while initialization SteamVR runtime. Error code is " << vr::VR_GetVRInitErrorAsSymbol(err) << std::endl;
            return false;
        } else {
            std::cout << "SteamVR runtime succesfully initialized" << std::endl;
        }

        // Initializing initial device list
        vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];
        m_VRSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, trackedDevicePose, vr::k_unMaxTrackedDeviceCount);
        for (uint32_t device_id = 0; device_id < vr::k_unMaxTrackedDeviceCount; device_id++){
            if (trackedDevicePose[device_id].bDeviceIsConnected){
                m_DeviceTable.insert({device_id, false});
            }
        }
        return true;
    }

    void VRSystem::OnUpdate(){

        ProcessEvents();
    }

    void VRSystem::ProcessEvents(){
        vr::VREvent_t vrEvent;
        while (m_VRSystem->PollNextEvent(&vrEvent, sizeof(vr::VREvent_t)))
        {
            switch (vrEvent.eventType)
            {
            case vr::VREvent_TrackedDeviceActivated:
                {
                    m_DeviceTable.insert({vrEvent.trackedDeviceIndex, false});

                    VRDeviceClass devClass = static_cast<VRDeviceClass>(m_VRSystem->GetTrackedDeviceClass(vrEvent.trackedDeviceIndex));
                    VRTrackedDeviceActivatedEvent event(vrEvent.trackedDeviceIndex, devClass);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_TrackedDeviceDeactivated:
                {
                    m_DeviceTable.erase(vrEvent.trackedDeviceIndex);

                    VRDeviceClass devClass = static_cast<VRDeviceClass>(m_VRSystem->GetTrackedDeviceClass(vrEvent.trackedDeviceIndex));
                    VRTrackedDeviceDeactivatedEvent event(vrEvent.trackedDeviceIndex, devClass);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_TrackedDeviceUpdated:
                {                    
                    VRDeviceClass devClass = static_cast<VRDeviceClass>(m_VRSystem->GetTrackedDeviceClass(vrEvent.trackedDeviceIndex));
                    VRTrackedDeviceUpdatedEvent event(vrEvent.trackedDeviceIndex, devClass);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_ButtonPress:
                {
                    VRDeviceClass devClass = static_cast<VRDeviceClass>(m_VRSystem->GetTrackedDeviceClass(vrEvent.trackedDeviceIndex));
                    VRButtonCode button = static_cast<VRButtonCode>(vrEvent.data.controller.button);
                    VRButtonPressEvent event(button, vrEvent.trackedDeviceIndex, devClass);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_ButtonUnpress:
                {
                    VRDeviceClass devClass = static_cast<VRDeviceClass>(m_VRSystem->GetTrackedDeviceClass(vrEvent.trackedDeviceIndex));
                    VRButtonCode button = static_cast<VRButtonCode>(vrEvent.data.controller.button);
                    VRButtonReleaseEvent event(button, vrEvent.trackedDeviceIndex, devClass);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_ButtonTouch:
                {
                    VRDeviceClass devClass = static_cast<VRDeviceClass>(m_VRSystem->GetTrackedDeviceClass(vrEvent.trackedDeviceIndex));
                    VRButtonCode button = static_cast<VRButtonCode>(vrEvent.data.controller.button);
                    VRButtonTouchEvent event(button, vrEvent.trackedDeviceIndex, devClass);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_ButtonUntouch:
                {
                    VRDeviceClass devClass = static_cast<VRDeviceClass>(m_VRSystem->GetTrackedDeviceClass(vrEvent.trackedDeviceIndex));
                    VRButtonCode button = static_cast<VRButtonCode>(vrEvent.data.controller.button);
                    VRButtonUntouchEvent event(button, vrEvent.trackedDeviceIndex, devClass);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_MouseMove:
                {
                    VRMouseMovedEvent event(vrEvent.data.mouse.x, vrEvent.data.mouse.y);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_MouseButtonDown:
                {
                    VRMouseButtonPressedEvent event(static_cast<VRMouseCode>(vrEvent.data.mouse.button));
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_MouseButtonUp:
                {
                    VRMouseButtonReleasedEvent event(static_cast<VRMouseCode>(vrEvent.data.mouse.button));
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_InputFocusCaptured:
                {
                    VRInputFocusCapturedEvent event(vrEvent.data.process.pid, vrEvent.data.process.oldPid);
                    m_EventCallback(event);
                    break;
                }
            case vr::VREvent_InputFocusReleased:
                {
                    VRInputFocusReleasedEvent event(vrEvent.data.process.pid, vrEvent.data.process.oldPid);
                    m_EventCallback(event);
                    break;
                }
            default:
                break;
            }
        }
    }
    
    void VRSystem::Shutdown(){
        vr::VR_Shutdown();
    }

    int VRSystem::GetUnboundId(VRDeviceClass deviceClass) const {

        for (auto device : m_DeviceTable){
            if ( (!device.second) && (deviceClass == static_cast<VRDeviceClass>( m_VRSystem->GetTrackedDeviceClass(device.first))) ){
                return device.first;
            }
        }

        std::cout << "No more available Id's for device of type " << deviceClass << std::endl;
        return -1;
    }

    void VRSystem::BindID(uint32_t id) {
        m_DeviceTable[id] = true;
    }

    void VRSystem::UnbindID(uint32_t id) {
        m_DeviceTable[id] = false;
    }

    glm::mat4 VRSystem::GetProjectionMatrix(EyeType eye, float nearClip, float farClip) const {
       return utilities::hmd44_to_mat4(m_VRSystem->GetProjectionMatrix(static_cast<vr::EVREye>(eye), nearClip, farClip));
    }

    glm::mat4 VRSystem::GetEyeToHeadTransform(EyeType eye) const {
        return utilities::hmd34_to_mat4(m_VRSystem->GetEyeToHeadTransform(static_cast<vr::EVREye>(eye)));
    }

    uint32_t VRSystem::GetRecomendedRenderWidth() const{
        uint32_t width = 0, height = 0;
        m_VRSystem->GetRecommendedRenderTargetSize(&width, &height);
        return width;
    }

    uint32_t VRSystem::GetRecomendedRenderHeight() const{
        uint32_t width = 0, height = 0;
        m_VRSystem->GetRecommendedRenderTargetSize(&width, &height);
        return height;
    }

    glm::mat4 VRSystem::GetHMDTransform() const{
        vr::TrackedDevicePose_t pose;
        vr::VRCompositor()->WaitGetPoses(&pose, 1, NULL, 0);
        return utilities::hmd34_to_mat4(pose.mDeviceToAbsoluteTracking);
    }

}