#ifndef VR_SYSTEM_H
#define VR_SYSTEM_H

#include <functional>
#include <unordered_map>

#include <openvr.h>
#include <glm/gtc/matrix_transform.hpp>

#include "rendering_engine/Events/Event.h"
#include "rendering_engine/VRCodes.h"

namespace rendering_engine{

class VRSystem {
public:
    static VRSystem* GetInstance();
    static void ReleaseInstance();

    bool Init();

    using EventCallbackFn = std::function<void(Event&)>;

    void OnUpdate();

    void Shutdown();

    void SetEventCallback(const EventCallbackFn& callback) {m_EventCallback = callback;}

    void* GetVRSystemPointer() const { return m_VRSystem; }

    int GetUnboundId(VRDeviceClass deviceClass) const;
    void BindID(uint32_t id);
    void UnbindID(uint32_t id);

    // **************************** //
    glm::mat4 GetProjectionMatrix(EyeType eye, float nearClip, float farClip) const;
    glm::mat4 GetEyeToHeadTransform(EyeType eye) const;
    uint32_t GetRecomendedRenderWidth() const;
    uint32_t GetRecomendedRenderHeight() const;
    glm::mat4 GetHMDTransform() const;

private:
    static VRSystem* s_Instance;
    static int s_Counter;
    static void AddRef();
    static void ReleaseRef();
    VRSystem();
    ~VRSystem();


    EventCallbackFn m_EventCallback;

    vr::IVRSystem* m_VRSystem;

    void ProcessEvents();

    std::unordered_map<uint32_t, bool> m_DeviceTable;

};

}

#endif // VR_SYSTEM_H