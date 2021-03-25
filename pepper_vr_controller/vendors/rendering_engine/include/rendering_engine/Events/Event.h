
/*
    This event system is taken from https://github.com/TheCherno/Hazel/blob/master/Hazel/src/Hazel/Events/Event.h
    and was featured in YouTube Game engine series made by channel The Cherno.
*/

#ifndef EVENT_H
#define EVENT_H

#define BIT(x) (1 << x)

#include <string>
#include <iostream>
#include <sstream>
#include <functional>


namespace rendering_engine{

	// Events are currently blocking, meaning when an event occurs it
	// immediately gets dispatched and must be dealt with right then an there.
	// For the future, a better strategy might be to buffer events in an event
	// bus and process them during the "event" part of the update stage.

	enum class EventType
	{
		None = 0,
		WindowClose, WindowResize, WindowFocus, WindowLostFocus, WindowMoved,
		AppTick, AppUpdate, AppRender,
		KeyPressed, KeyReleased, KeyTyped,
		MouseButtonPressed, MouseButtonReleased, MouseMoved, MouseScrolled,
		VRTrackedDeviceActivated, VRTrackedDeviceDeactivated, VRTrackedDeviceUpdated, 
		VRButtonPress, VRButtonRelease, VRButtonTouch, VRButtonUntouch, 
		VRMouseMoved, VRMouseButtonPressed, VRMouseButtonReleased, 
		VRInputFocusCaptured, VRInputFocusReleased,
		CanvasPointerMoved
	};

	enum EventCategory
	{
		None = 0,
		EventCategoryApplication    	= BIT(0),
		EventCategoryInput          	= BIT(1),
		EventCategoryKeyboard       	= BIT(2),
		EventCategoryMouse          	= BIT(3),
		EventCategoryMouseButton    	= BIT(4),
		EventCategoryVR					= BIT(5),
		EventCategoryVRTrackedDevice	= BIT(6),
		EventCategoryVRController		= BIT(7),
		EventCategoryVRMouse			= BIT(8),
		EventCategoryCanvasPointer		= BIT(9)
	};

#define EVENT_CLASS_TYPE(type) static EventType GetStaticType() { return EventType::type; }\
								virtual EventType GetEventType() const override { return GetStaticType(); }\
								virtual const char* GetName() const override { return #type; }

#define EVENT_CLASS_CATEGORY(category) virtual int GetCategoryFlags() const override { return category; }

	class Event
	{
	public:
		bool Handled = false;

		virtual EventType GetEventType() const = 0;
		virtual const char* GetName() const = 0;
		virtual int GetCategoryFlags() const = 0;
		virtual std::string ToString() const { return GetName(); }

		bool IsInCategory(EventCategory category)
		{
			return GetCategoryFlags() & category;
		}
	};

	class EventDispatcher
	{
	public:
		EventDispatcher(Event& event)
			: m_Event(event)
		{
		}
		
		// F will be deduced by the compiler
		template<typename T, typename F>
		bool Dispatch(const F& func)
		{
			if (m_Event.GetEventType() == T::GetStaticType())
			{
				m_Event.Handled = func(static_cast<T&>(m_Event));
				return true;
			}
			return false;
		}
	private:
		Event& m_Event;
	};

	inline std::ostream& operator<<(std::ostream& os, const Event& e)
	{
		return os << e.ToString();
	}

}

#endif // EVENT_H