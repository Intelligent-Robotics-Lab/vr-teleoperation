#ifndef KEY_CODES_H
#define KEY_CODES_H

#include <ostream>

namespace rendering_engine{

	typedef enum class KeyCode : uint16_t
	{
		// From glfw3.h
		Space               = 32,
		Apostrophe          = 39, /* ' */
		Comma               = 44, /* , */
		Minus               = 45, /* - */
		Period              = 46, /* . */
		Slash               = 47, /* / */

		D0                  = 48, /* 0 */
		D1                  = 49, /* 1 */
		D2                  = 50, /* 2 */
		D3                  = 51, /* 3 */
		D4                  = 52, /* 4 */
		D5                  = 53, /* 5 */
		D6                  = 54, /* 6 */
		D7                  = 55, /* 7 */
		D8                  = 56, /* 8 */
		D9                  = 57, /* 9 */

		Semicolon           = 59, /* ; */
		Equal               = 61, /* = */

		A                   = 65,
		B                   = 66,
		C                   = 67,
		D                   = 68,
		E                   = 69,
		F                   = 70,
		G                   = 71,
		H                   = 72,
		I                   = 73,
		J                   = 74,
		K                   = 75,
		L                   = 76,
		M                   = 77,
		N                   = 78,
		O                   = 79,
		P                   = 80,
		Q                   = 81,
		R                   = 82,
		S                   = 83,
		T                   = 84,
		U                   = 85,
		V                   = 86,
		W                   = 87,
		X                   = 88,
		Y                   = 89,
		Z                   = 90,

		LeftBracket         = 91,  /* [ */
		Backslash           = 92,  /* \ */
		RightBracket        = 93,  /* ] */
		GraveAccent         = 96,  /* ` */

		World1              = 161, /* non-US #1 */
		World2              = 162, /* non-US #2 */

		/* Function keys */
		Escape              = 256,
		Enter               = 257,
		Tab                 = 258,
		Backspace           = 259,
		Insert              = 260,
		Delete              = 261,
		Right               = 262,
		Left                = 263,
		Down                = 264,
		Up                  = 265,
		PageUp              = 266,
		PageDown            = 267,
		Home                = 268,
		End                 = 269,
		CapsLock            = 280,
		ScrollLock          = 281,
		NumLock             = 282,
		PrintScreen         = 283,
		Pause               = 284,
		F1                  = 290,
		F2                  = 291,
		F3                  = 292,
		F4                  = 293,
		F5                  = 294,
		F6                  = 295,
		F7                  = 296,
		F8                  = 297,
		F9                  = 298,
		F10                 = 299,
		F11                 = 300,
		F12                 = 301,
		F13                 = 302,
		F14                 = 303,
		F15                 = 304,
		F16                 = 305,
		F17                 = 306,
		F18                 = 307,
		F19                 = 308,
		F20                 = 309,
		F21                 = 310,
		F22                 = 311,
		F23                 = 312,
		F24                 = 313,
		F25                 = 314,

		/* Keypad */
		KP0                 = 320,
		KP1                 = 321,
		KP2                 = 322,
		KP3                 = 323,
		KP4                 = 324,
		KP5                 = 325,
		KP6                 = 326,
		KP7                 = 327,
		KP8                 = 328,
		KP9                 = 329,
		KPDecimal           = 330,
		KPDivide            = 331,
		KPMultiply          = 332,
		KPSubtract          = 333,
		KPAdd               = 334,
		KPEnter             = 335,
		KPEqual             = 336,

		LeftShift           = 340,
		LeftControl         = 341,
		LeftAlt             = 342,
		LeftSuper           = 343,
		RightShift          = 344,
		RightControl        = 345,
		RightAlt            = 346,
		RightSuper          = 347,
		Menu                = 348
	} Key;

	inline std::ostream& operator<<(std::ostream& os, KeyCode keyCode)
	{
		os << static_cast<int32_t>(keyCode);
		return os;
	}
}

// From glfw3.h
#define RE_KEY_SPACE           ::rendering_engine::Key::Space
#define RE_KEY_APOSTROPHE      ::rendering_engine::Key::Apostrophe    /* ' */
#define RE_KEY_COMMA           ::rendering_engine::Key::Comma         /* , */
#define RE_KEY_MINUS           ::rendering_engine::Key::Minus         /* - */
#define RE_KEY_PERIOD          ::rendering_engine::Key::Period        /* . */
#define RE_KEY_SLASH           ::rendering_engine::Key::Slash         /* / */
#define RE_KEY_0               ::rendering_engine::Key::D0
#define RE_KEY_1               ::rendering_engine::Key::D1
#define RE_KEY_2               ::rendering_engine::Key::D2
#define RE_KEY_3               ::rendering_engine::Key::D3
#define RE_KEY_4               ::rendering_engine::Key::D4
#define RE_KEY_5               ::rendering_engine::Key::D5
#define RE_KEY_6               ::rendering_engine::Key::D6
#define RE_KEY_7               ::rendering_engine::Key::D7
#define RE_KEY_8               ::rendering_engine::Key::D8
#define RE_KEY_9               ::rendering_engine::Key::D9
#define RE_KEY_SEMICOLON       ::rendering_engine::Key::Semicolon     /* ; */
#define RE_KEY_EQUAL           ::rendering_engine::Key::Equal         /* = */
#define RE_KEY_A               ::rendering_engine::Key::A
#define RE_KEY_B               ::rendering_engine::Key::B
#define RE_KEY_C               ::rendering_engine::Key::C
#define RE_KEY_D               ::rendering_engine::Key::D
#define RE_KEY_E               ::rendering_engine::Key::E
#define RE_KEY_F               ::rendering_engine::Key::F
#define RE_KEY_G               ::rendering_engine::Key::G
#define RE_KEY_H               ::rendering_engine::Key::H
#define RE_KEY_I               ::rendering_engine::Key::I
#define RE_KEY_J               ::rendering_engine::Key::J
#define RE_KEY_K               ::rendering_engine::Key::K
#define RE_KEY_L               ::rendering_engine::Key::L
#define RE_KEY_M               ::rendering_engine::Key::M
#define RE_KEY_N               ::rendering_engine::Key::N
#define RE_KEY_O               ::rendering_engine::Key::O
#define RE_KEY_P               ::rendering_engine::Key::P
#define RE_KEY_Q               ::rendering_engine::Key::Q
#define RE_KEY_R               ::rendering_engine::Key::R
#define RE_KEY_S               ::rendering_engine::Key::S
#define RE_KEY_T               ::rendering_engine::Key::T
#define RE_KEY_U               ::rendering_engine::Key::U
#define RE_KEY_V               ::rendering_engine::Key::V
#define RE_KEY_W               ::rendering_engine::Key::W
#define RE_KEY_X               ::rendering_engine::Key::X
#define RE_KEY_Y               ::rendering_engine::Key::Y
#define RE_KEY_Z               ::rendering_engine::Key::Z
#define RE_KEY_LEFT_BRACKET    ::rendering_engine::Key::LeftBracket   /* [ */
#define RE_KEY_BACKSLASH       ::rendering_engine::Key::Backslash     /* \ */
#define RE_KEY_RIGHT_BRACKET   ::rendering_engine::Key::RightBracket  /* ] */
#define RE_KEY_GRAVE_ACCENT    ::rendering_engine::Key::GraveAccent   /* ` */
#define RE_KEY_WORLD_1         ::rendering_engine::Key::World1        /* non-US #1 */
#define RE_KEY_WORLD_2         ::rendering_engine::Key::World2        /* non-US #2 */

/* Function keys */
#define RE_KEY_ESCAPE          ::rendering_engine::Key::Escape
#define RE_KEY_ENTER           ::rendering_engine::Key::Enter
#define RE_KEY_TAB             ::rendering_engine::Key::Tab
#define RE_KEY_BACKSPACE       ::rendering_engine::Key::Backspace
#define RE_KEY_INSERT          ::rendering_engine::Key::Insert
#define RE_KEY_DELETE          ::rendering_engine::Key::Delete
#define RE_KEY_RIGHT           ::rendering_engine::Key::Right
#define RE_KEY_LEFT            ::rendering_engine::Key::Left
#define RE_KEY_DOWN            ::rendering_engine::Key::Down
#define RE_KEY_UP              ::rendering_engine::Key::Up
#define RE_KEY_PAGE_UP         ::rendering_engine::Key::PageUp
#define RE_KEY_PAGE_DOWN       ::rendering_engine::Key::PageDown
#define RE_KEY_HOME            ::rendering_engine::Key::Home
#define RE_KEY_END             ::rendering_engine::Key::End
#define RE_KEY_CAPS_LOCK       ::rendering_engine::Key::CapsLock
#define RE_KEY_SCROLL_LOCK     ::rendering_engine::Key::ScrollLock
#define RE_KEY_NUM_LOCK        ::rendering_engine::Key::NumLock
#define RE_KEY_PRINT_SCREEN    ::rendering_engine::Key::PrintScreen
#define RE_KEY_PAUSE           ::rendering_engine::Key::Pause
#define RE_KEY_F1              ::rendering_engine::Key::F1
#define RE_KEY_F2              ::rendering_engine::Key::F2
#define RE_KEY_F3              ::rendering_engine::Key::F3
#define RE_KEY_F4              ::rendering_engine::Key::F4
#define RE_KEY_F5              ::rendering_engine::Key::F5
#define RE_KEY_F6              ::rendering_engine::Key::F6
#define RE_KEY_F7              ::rendering_engine::Key::F7
#define RE_KEY_F8              ::rendering_engine::Key::F8
#define RE_KEY_F9              ::rendering_engine::Key::F9
#define RE_KEY_F10             ::rendering_engine::Key::F10
#define RE_KEY_F11             ::rendering_engine::Key::F11
#define RE_KEY_F12             ::rendering_engine::Key::F12
#define RE_KEY_F13             ::rendering_engine::Key::F13
#define RE_KEY_F14             ::rendering_engine::Key::F14
#define RE_KEY_F15             ::rendering_engine::Key::F15
#define RE_KEY_F16             ::rendering_engine::Key::F16
#define RE_KEY_F17             ::rendering_engine::Key::F17
#define RE_KEY_F18             ::rendering_engine::Key::F18
#define RE_KEY_F19             ::rendering_engine::Key::F19
#define RE_KEY_F20             ::rendering_engine::Key::F20
#define RE_KEY_F21             ::rendering_engine::Key::F21
#define RE_KEY_F22             ::rendering_engine::Key::F22
#define RE_KEY_F23             ::rendering_engine::Key::F23
#define RE_KEY_F24             ::rendering_engine::Key::F24
#define RE_KEY_F25             ::rendering_engine::Key::F25

/* Keypad */
#define RE_KEY_KP_0            ::rendering_engine::Key::KP0
#define RE_KEY_KP_1            ::rendering_engine::Key::KP1
#define RE_KEY_KP_2            ::rendering_engine::Key::KP2
#define RE_KEY_KP_3            ::rendering_engine::Key::KP3
#define RE_KEY_KP_4            ::rendering_engine::Key::KP4
#define RE_KEY_KP_5            ::rendering_engine::Key::KP5
#define RE_KEY_KP_6            ::rendering_engine::Key::KP6
#define RE_KEY_KP_7            ::rendering_engine::Key::KP7
#define RE_KEY_KP_8            ::rendering_engine::Key::KP8
#define RE_KEY_KP_9            ::rendering_engine::Key::KP9
#define RE_KEY_KP_DECIMAL      ::rendering_engine::Key::KPDecimal
#define RE_KEY_KP_DIVIDE       ::rendering_engine::Key::KPDivide
#define RE_KEY_KP_MULTIPLY     ::rendering_engine::Key::KPMultiply
#define RE_KEY_KP_SUBTRACT     ::rendering_engine::Key::KPSubtract
#define RE_KEY_KP_ADD          ::rendering_engine::Key::KPAdd
#define RE_KEY_KP_ENTER        ::rendering_engine::Key::KPEnter
#define RE_KEY_KP_EQUAL        ::rendering_engine::Key::KPEqual

#define RE_KEY_LEFT_SHIFT      ::rendering_engine::Key::LeftShift
#define RE_KEY_LEFT_CONTROL    ::rendering_engine::Key::LeftControl
#define RE_KEY_LEFT_ALT        ::rendering_engine::Key::LeftAlt
#define RE_KEY_LEFT_SUPER      ::rendering_engine::Key::LeftSuper
#define RE_KEY_RIGHT_SHIFT     ::rendering_engine::Key::RightShift
#define RE_KEY_RIGHT_CONTROL   ::rendering_engine::Key::RightControl
#define RE_KEY_RIGHT_ALT       ::rendering_engine::Key::RightAlt
#define RE_KEY_RIGHT_SUPER     ::rendering_engine::Key::RightSuper
#define RE_KEY_MENU            ::rendering_engine::Key::Menu

#endif // KEY_CODES_H