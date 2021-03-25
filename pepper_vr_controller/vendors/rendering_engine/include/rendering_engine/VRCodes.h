#ifndef VR_CODES_H
#define VR_CODES_H

#include <ostream>

namespace rendering_engine{
    typedef enum class VRDeviceClassCodes : uint16_t
    {
		Invalid             = 0,	// the ID was not valid.
		HMD                 = 1,	// Head-Mounted Displays
		Controller          = 2,	// Tracked controllers
		Tracker      		= 3,	// Generic trackers, similar to controllers
		TrackingReference   = 4,	// Camera and base stations that serve as tracking reference points
		DisplayRedirect     = 5		// Accessories that aren't necessarily tracked themselves, but may redirect video output from other tracked devices
    } VRDeviceClass;

	inline std::ostream& operator<<(std::ostream& os, VRDeviceClassCodes vrDeviceClassCode)
	{
		os << static_cast<int32_t>(vrDeviceClassCode);
		return os;
	}

	// typedef enum class VRDeviceRoleCodes : uint16_t
    // {
	// 	Any             		= 0,
	// 	RightController     	= 1,
	// 	LeftController      	= 2,
	// 	RightElbowTracker   	= 3,
	// 	LeftElbowTracker   		= 4,
	// 	RightShoulderTracker   	= 5,
	// 	LeftShoulderTracker   	= 6,
	// 	WaistTracker			= 7,
	// 	RightKneeTracker   		= 8,
	// 	LeftKneeTracker   		= 9,
	// 	RightFootTracker   		= 10,
	// 	LeftFootTracker   		= 11
    // } VRDeviceRole;

	// inline std::ostream& operator<<(std::ostream& os, VRDeviceRoleCodes vrDeviceRoleCode)
	// {
	// 	os << static_cast<int32_t>(vrDeviceRoleCode);
	// 	return os;
	// }

	typedef enum class VRButtonCode : uint16_t
	{
		System				= 0, // The system button. These events are not visible to applications and are used internally to bring up the Steam Overlay or the Steam Client.
		ApplicationMenu		= 1, // The application menu button
		Grip				= 2, // The grip button
		Axis0				= 3, // Axis 0
		Axis1				= 4, // Axis 0
		Axis2				= 5, // Axis 0
		Axis3				= 6, // Axis 0
		Axis4				= 7, // Axis 0

		TouchPad 			= Axis0,
		Trigger				= Axis1
	} VRButton;

	inline std::ostream& operator<<(std::ostream& os, VRButtonCode vrButtonCode)
	{
		os << static_cast<int32_t>(vrButtonCode);
		return os;
	}

	typedef enum class VRMouseCode : uint16_t
	{
		Left				= 0, 
		Right				= 1, 
		Middle				= 2, 

	} VRMouse;

	inline std::ostream& operator<<(std::ostream& os, VRMouseCode vrMouseCode)
	{
		os << static_cast<int32_t>(vrMouseCode);
		return os;
	}

	// From openvr
	enum EyeType : uint16_t{
        LeftEye  = 0,
        RightEye = 1
    };

	enum Hand : uint16_t{
        Left  = 0,
        Right = 1
    };

}

#define RE_VR_DEVICE_CLASS_INVALID				::rendering_engine::VRDeviceClass::Invalid
#define RE_VR_DEVICE_CLASS_HMD					::rendering_engine::VRDeviceClass::HMD
#define RE_VR_DEVICE_CLASS_CONTROLLER			::rendering_engine::VRDeviceClass::Controller
#define RE_VR_DEVICE_CLASS_TRACKER				::rendering_engine::VRDeviceClass::Tracker
#define RE_VR_DEVICE_CLASS_TRACKING_REFERENCE	::rendering_engine::VRDeviceClass::TrackingReference
#define RE_VR_DEVICE_CLASS_DISPLAY_REDIRECT		::rendering_engine::VRDeviceClass::DisplayRedirect

#define RE_VR_BUTTON_SYSTEM						::rendering_engine::VRButton::System
#define RE_VR_BUTTON_APPLICATION_MENU			::rendering_engine::VRButton::ApplicationMenu
#define RE_VR_BUTTON_GRIP						::rendering_engine::VRButton::Grip
#define RE_VR_BUTTON_AXIS0						::rendering_engine::VRButton::Axis0
#define RE_VR_BUTTON_AXIS1						::rendering_engine::VRButton::Axis1
#define RE_VR_BUTTON_AXIS2						::rendering_engine::VRButton::Axis2
#define RE_VR_BUTTON_AXIS3						::rendering_engine::VRButton::Axis3
#define RE_VR_BUTTON_AXIS4						::rendering_engine::VRButton::Axis4

#define RE_VR_MOUSE_LEFT						::rendering_engine::VRMouse::Left
#define RE_VR_MOUSE_RIGHT						::rendering_engine::VRMouse::Right
#define RE_VR_MOUSE_MIDDLE						::rendering_engine::VRMouse::Middle

// #define RE_VR_DEVICE_ROLE_ANY					::rendering_engine::VRDeviceRole::Any
// #define RE_VR_DEVICE_ROLE_RIGHT_CONTROLLER		::rendering_engine::VRDeviceRole::RightController
// #define RE_VR_DEVICE_ROLE_LEFT_CONTROLLER		::rendering_engine::VRDeviceRole::LeftController
// #define RE_VR_DEVICE_ROLE_RIGHT_ELBOW			::rendering_engine::VRDeviceRole::RightElbowTracker
// #define RE_VR_DEVICE_ROLE_LEFT_ELBOW			::rendering_engine::VRDeviceRole::LeftElbowTracker
// #define RE_VR_DEVICE_ROLE_RIGHT_SHOULDER		::rendering_engine::VRDeviceRole::RightShoulderTracker
// #define RE_VR_DEVICE_ROLE_LEFT_SHOULDER			::rendering_engine::VRDeviceRole::LeftShoulderTracker
// #define RE_VR_DEVICE_ROLE_WAIST					::rendering_engine::VRDeviceRole::WaistTracker
// #define RE_VR_DEVICE_ROLE_RIGHT_KNEE			::rendering_engine::VRDeviceRole::RightKneeTracker
// #define RE_VR_DEVICE_ROLE_LEFT_KNEE				::rendering_engine::VRDeviceRole::LeftKneeTracker
// #define RE_VR_DEVICE_ROLE_RIGHT_FOOT			::rendering_engine::VRDeviceRole::RightFootTracker
// #define RE_VR_DEVICE_ROLE_LEFT_FOOT				::rendering_engine::VRDeviceRole::LeftFootTracker

#endif // VR_CODES_H