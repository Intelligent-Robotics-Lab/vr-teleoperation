#ifndef VR_EVENT_H
#define VR_EVENT_H

#include "rendering_engine/Events/Event.h"
#include "rendering_engine/VRCodes.h"

namespace rendering_engine{

    class VRTrackedDeviceEvent : public Event
    {
    public:
        uint32_t getDeviceId() const { return m_Id; }
        VRDeviceClass getDeviceClass() const { return m_DeviceClass; }

        EVENT_CLASS_CATEGORY(EventCategoryVRTrackedDevice | EventCategoryVR)
    protected:
        VRTrackedDeviceEvent(uint32_t id, VRDeviceClass devClass)
            : m_Id(id), m_DeviceClass(devClass) {}

        uint32_t m_Id;
        VRDeviceClass m_DeviceClass;
    };


    class VRTrackedDeviceActivatedEvent : public VRTrackedDeviceEvent
    {
    public:
        VRTrackedDeviceActivatedEvent(uint32_t id, VRDeviceClass devClass)
            : VRTrackedDeviceEvent(id, devClass) {}

        std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRTrackedDeviceActivatedEvent: id" << m_Id << " of class " << m_DeviceClass;
			return ss.str();
		}
        EVENT_CLASS_TYPE(VRTrackedDeviceActivated)
    };
    

    class VRTrackedDeviceDeactivatedEvent : public VRTrackedDeviceEvent
    {
    public:
        VRTrackedDeviceDeactivatedEvent(uint32_t id, VRDeviceClass devClass)
            : VRTrackedDeviceEvent(id, devClass) {}

        std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRTrackedDeviceDeactivatedEvent: id" << m_Id << " of class " << m_DeviceClass;
			return ss.str();
		}
        EVENT_CLASS_TYPE(VRTrackedDeviceDeactivated)
    };


    class VRTrackedDeviceUpdatedEvent : public VRTrackedDeviceEvent
    {
    public:
        VRTrackedDeviceUpdatedEvent(uint32_t id, VRDeviceClass devClass)
            : VRTrackedDeviceEvent(id, devClass) {}

        std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRTrackedDeviceUpdatedEvent: id" << m_Id << " of class " << m_DeviceClass;
			return ss.str();
		}
        EVENT_CLASS_TYPE(VRTrackedDeviceUpdated)
    };


    class VRButtonEvent : public Event
    {
    public:
        VRButton getButton() const { return m_ButtonId; }
        uint32_t getDeviceId() const { return m_Id; }
        VRDeviceClass getDeviceClass() const { return m_DeviceClass; }

        EVENT_CLASS_CATEGORY(EventCategoryVR | EventCategoryVRTrackedDevice | EventCategoryVRController)

    protected:
        VRButtonEvent(VRButton button, uint32_t id, VRDeviceClass devClass)
            : m_ButtonId(button), m_Id(id), m_DeviceClass(devClass) {}

        VRButton m_ButtonId;
        uint32_t m_Id;
        VRDeviceClass m_DeviceClass;
    };


    class VRButtonPressEvent : public VRButtonEvent
    {
    public:
        VRButtonPressEvent(VRButton button, uint32_t id, VRDeviceClass devClass)
            : VRButtonEvent(button, id, devClass) {}

        std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRButtonPressEvent: Button: " << m_ButtonId << " DeviceId: " << m_Id << " of class " << m_DeviceClass;
			return ss.str();
		}

        EVENT_CLASS_TYPE(VRButtonPress)
    };

    class VRButtonReleaseEvent : public VRButtonEvent
    {
    public:
        VRButtonReleaseEvent(VRButton button, uint32_t id, VRDeviceClass devClass)
            : VRButtonEvent(button, id, devClass) {}

        std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRButtonReleaseEvent: Button: " << m_ButtonId << " DeviceId: " << m_Id << " of class " << m_DeviceClass;
			return ss.str();
		}

        EVENT_CLASS_TYPE(VRButtonRelease)
    };


    class VRButtonTouchEvent : public VRButtonEvent
    {
    public:
        VRButtonTouchEvent(VRButton button, uint32_t id, VRDeviceClass devClass)
            : VRButtonEvent(button, id, devClass) {}

        std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRButtonTouchEvent: Button: " << m_ButtonId << " DeviceId: " << m_Id << " of class " << m_DeviceClass;
			return ss.str();
		}

        EVENT_CLASS_TYPE(VRButtonTouch)
    };


    class VRButtonUntouchEvent : public VRButtonEvent
    {
    public:
        VRButtonUntouchEvent(VRButton button, uint32_t id, VRDeviceClass devClass)
            : VRButtonEvent(button, id, devClass) {}

        std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRButtonUntouchEvent: Button: " << m_ButtonId << " DeviceId: " << m_Id << " of class " << m_DeviceClass;
			return ss.str();
		}

        EVENT_CLASS_TYPE(VRButtonUntouch)
    };


    class VRMouseMovedEvent : public Event
    {
    public:
       	VRMouseMovedEvent(float x, float y)
			: m_MouseX(x), m_MouseY(y) {}

		float GetX() const { return m_MouseX; }
		float GetY() const { return m_MouseY; }

		std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRMouseMovedEvent: " << m_MouseX << ", " << m_MouseY;
			return ss.str();
		}

		EVENT_CLASS_TYPE(VRMouseMoved)
		EVENT_CLASS_CATEGORY(EventCategoryVR | EventCategoryVRMouse)
	private:
		float m_MouseX, m_MouseY; 
    };

	
    class VRMouseButtonEvent : public Event
	{
	public:
		inline VRMouseCode GetMouseButton() const { return m_Button; }

		EVENT_CLASS_CATEGORY(EventCategoryVRMouse | EventCategoryVR | EventCategoryInput)
	protected:
		VRMouseButtonEvent(VRMouseCode button)
			: m_Button(button) {}

		VRMouseCode m_Button;
	};

	class VRMouseButtonPressedEvent : public VRMouseButtonEvent
	{
	public:
		VRMouseButtonPressedEvent(VRMouseCode button)
			: VRMouseButtonEvent(button) {}

		std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRMouseButtonPressedEvent: " << m_Button;
			return ss.str();
		}

		EVENT_CLASS_TYPE(VRMouseButtonPressed)
	};

    class VRMouseButtonReleasedEvent : public VRMouseButtonEvent
	{
	public:
		VRMouseButtonReleasedEvent(VRMouseCode button)
			: VRMouseButtonEvent(button) {}

		std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRMouseButtonReleasedEvent: " << m_Button;
			return ss.str();
		}

		EVENT_CLASS_TYPE(VRMouseButtonReleased)
	};

    class VRInputFocusEvent : public Event
    {
        public:
            uint32_t GetPid() const { return m_Pid; }
            uint32_t GetOldPid() const { return m_OldPid; }

            EVENT_CLASS_CATEGORY(EventCategoryVR)

        protected:
            VRInputFocusEvent(uint32_t pid, uint32_t oldPid)
                : m_Pid(pid), m_OldPid(oldPid) {}

            uint32_t m_Pid;
            uint32_t m_OldPid;
    };

    class VRInputFocusCapturedEvent : public VRInputFocusEvent
    {
    public:
        VRInputFocusCapturedEvent(uint32_t pid, uint32_t oldPid)
            : VRInputFocusEvent(pid, oldPid) {}

		std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRInputFocusCapturedEvent: PID=" << m_Pid << " OldPID=" << m_OldPid;
			return ss.str();
		}

        EVENT_CLASS_TYPE(VRInputFocusCaptured)
    };

    class VRInputFocusReleasedEvent : public VRInputFocusEvent
    {
    public:
        VRInputFocusReleasedEvent(uint32_t pid, uint32_t oldPid)
            : VRInputFocusEvent(pid, oldPid) {}

		std::string ToString() const override
		{
			std::stringstream ss;
			ss << "VRInputFocusReleasedEvent: PID=" << m_Pid << " OldPID=" << m_OldPid;
			return ss.str();
		}

        EVENT_CLASS_TYPE(VRInputFocusReleased)
    };
}

#endif // VR_EVENT_H