#ifndef MOUSE_CODES_H
#define MOUSE_CODES_H

#include <ostream>

namespace rendering_engine{
    typedef enum class MouseCode : uint16_t
	{
		// From glfw3.h
		Button0                = 0,
		Button1                = 1,
		Button2                = 2,
		Button3                = 3,
		Button4                = 4,
		Button5                = 5,
		Button6                = 6,
		Button7                = 7,

		ButtonLast             = Button7,
		ButtonLeft             = Button0,
		ButtonRight            = Button1,
		ButtonMiddle           = Button2
	} Mouse;
	
	inline std::ostream& operator<<(std::ostream& os, MouseCode mouseCode)
	{
		os << static_cast<int32_t>(mouseCode);
		return os;
	}
}

#define RE_MOUSE_BUTTON_0      ::rendering_engine::Mouse::Button0
#define RE_MOUSE_BUTTON_1      ::rendering_engine::Mouse::Button1
#define RE_MOUSE_BUTTON_2      ::rendering_engine::Mouse::Button2
#define RE_MOUSE_BUTTON_3      ::rendering_engine::Mouse::Button3
#define RE_MOUSE_BUTTON_4      ::rendering_engine::Mouse::Button4
#define RE_MOUSE_BUTTON_5      ::rendering_engine::Mouse::Button5
#define RE_MOUSE_BUTTON_6      ::rendering_engine::Mouse::Button6
#define RE_MOUSE_BUTTON_7      ::rendering_engine::Mouse::Button7
#define RE_MOUSE_BUTTON_LAST   ::rendering_engine::Mouse::ButtonLast
#define RE_MOUSE_BUTTON_LEFT   ::rendering_engine::Mouse::ButtonLeft
#define RE_MOUSE_BUTTON_RIGHT  ::rendering_engine::Mouse::ButtonRight
#define RE_MOUSE_BUTTON_MIDDLE ::rendering_engine::Mouse::ButtonMiddle


#endif // MOUSE_CODES_H