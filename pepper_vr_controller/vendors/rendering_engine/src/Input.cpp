#include "rendering_engine/Input.h"

#include "rendering_engine/Application.h"
#include "GLFW/glfw3.h"

namespace rendering_engine{
    Input* Input::s_Instance = new Input();
    
    bool Input::IsKeyPressedImpl(KeyCode key){
        auto window = static_cast<GLFWwindow*>(Application::Get().GetWindow().GetWindowPointer());
        auto state = glfwGetKey(window, static_cast<int32_t>(key));
        return state == GLFW_PRESS || state == GLFW_REPEAT;
    }

	bool Input::IsMouseButtonPressedImpl(MouseCode button){
		auto window = static_cast<GLFWwindow*>(Application::Get().GetWindow().GetWindowPointer());
		auto state = glfwGetMouseButton(window, static_cast<int32_t>(button));
		return state == GLFW_PRESS;
    }

	std::pair<float, float> Input::GetMousePositionImpl(){
		auto window = static_cast<GLFWwindow*>(Application::Get().GetWindow().GetWindowPointer());
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		return { (float)xpos, (float)ypos };
    }

	float Input::GetMouseXImpl(){
        auto p = GetMousePositionImpl();
		return p.first;
    }

	float Input::GetMouseYImpl(){
		auto p = GetMousePositionImpl();
		return p.second;
    }

}