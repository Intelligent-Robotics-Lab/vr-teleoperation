#include "rendering_engine/GUI.h"

#include <imgui.h>

#include "rendering_engine/KeyCodes.h"

#include <examples/imgui_impl_opengl3.h>

namespace rendering_engine {
    
    GUI::GUI(){
		IMGUI_CHECKVERSION();
        ImGui::CreateContext();
		ImGui::StyleColorsDark();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
		io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
		io.BackendFlags |= ImGuiBackendFlags_HasSetMousePos;

		io.MouseDrawCursor = true;

		io.KeyMap[ImGuiKey_Tab] = (int)RE_KEY_TAB;
		io.KeyMap[ImGuiKey_LeftArrow] = (int)RE_KEY_LEFT;
		io.KeyMap[ImGuiKey_RightArrow] = (int)RE_KEY_RIGHT;
		io.KeyMap[ImGuiKey_UpArrow] = (int)RE_KEY_UP;
		io.KeyMap[ImGuiKey_DownArrow] = (int)RE_KEY_DOWN;
		io.KeyMap[ImGuiKey_PageUp] = (int)RE_KEY_PAGE_UP;
		io.KeyMap[ImGuiKey_PageDown] = (int)RE_KEY_PAGE_DOWN;
		io.KeyMap[ImGuiKey_Home] = (int)RE_KEY_HOME;
		io.KeyMap[ImGuiKey_End] = (int)RE_KEY_END;
		io.KeyMap[ImGuiKey_Insert] = (int)RE_KEY_INSERT;
		io.KeyMap[ImGuiKey_Delete] = (int)RE_KEY_DELETE;
		io.KeyMap[ImGuiKey_Backspace] = (int)RE_KEY_BACKSPACE;
		io.KeyMap[ImGuiKey_Space] = (int)RE_KEY_SPACE;
		io.KeyMap[ImGuiKey_Enter] = (int)RE_KEY_ENTER;
		io.KeyMap[ImGuiKey_Escape] = (int)RE_KEY_ESCAPE;
		io.KeyMap[ImGuiKey_A] = (int)RE_KEY_A;
		io.KeyMap[ImGuiKey_C] = (int)RE_KEY_C;
		io.KeyMap[ImGuiKey_V] = (int)RE_KEY_V;
		io.KeyMap[ImGuiKey_X] = (int)RE_KEY_X;
		io.KeyMap[ImGuiKey_Y] = (int)RE_KEY_Y;
		io.KeyMap[ImGuiKey_Z] = (int)RE_KEY_Z;

		//Application& app = Application::Get();
		io.DisplaySize = ImVec2(1280.0f, 720.0f);

		ImGui_ImplOpenGL3_Init("#version 330");
    }

    GUI::~GUI(){

    }


	void GUI::Begin(Timestep ts){
		ImGui_ImplOpenGL3_NewFrame();
		ImGui::NewFrame();

		ImGuiIO& io = ImGui::GetIO(); (void)io;
		io.DeltaTime = ts;
	}

	void GUI::End(){
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}

    void GUI::SetClickInputStatus(bool click){
		ImGuiIO& io = ImGui::GetIO();

		if (io.MouseDown[0] ^ click)
			io.MouseDown[0] = click;
	}


    void GUI::OnEvent(Event& event){
        EventDispatcher dispatcher(event);
        dispatcher.Dispatch<MouseButtonPressedEvent>(std::bind(&GUI::OnMouseButtonPressedEvent, this, std::placeholders::_1));
        dispatcher.Dispatch<MouseButtonReleasedEvent>(std::bind(&GUI::OnMouseButtonReleaseEvent, this, std::placeholders::_1));
        dispatcher.Dispatch<MouseMovedEvent>(std::bind(&GUI::OnMouseMovedEvent, this, std::placeholders::_1));
        dispatcher.Dispatch<MouseScrolledEvent>(std::bind(&GUI::OnMouseScrollEvent, this, std::placeholders::_1));
        dispatcher.Dispatch<KeyPressedEvent>(std::bind(&GUI::OnKeyPressedEvent, this, std::placeholders::_1));
        dispatcher.Dispatch<KeyReleasedEvent>(std::bind(&GUI::OnKeyReleasedEvent, this, std::placeholders::_1));
        dispatcher.Dispatch<KeyTypedEvent>(std::bind(&GUI::OnKeyTypedEvent, this, std::placeholders::_1));
        dispatcher.Dispatch<WindowResizeEvent>(std::bind(&GUI::OnWindowResizeEvent, this, std::placeholders::_1));
		dispatcher.Dispatch<CanvasPointerMovedEvent>(std::bind(&GUI::OnCanvasPointerMovedEvent, this, std::placeholders::_1));
    }

    bool GUI::OnMouseButtonPressedEvent(MouseButtonPressedEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		io.MouseDown[(int)e.GetMouseButton()] = true;

        return false;
    }

    bool GUI::OnMouseButtonReleaseEvent(MouseButtonReleasedEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		io.MouseDown[(int)e.GetMouseButton()] = false;

        return false;
    }

    bool GUI::OnMouseMovedEvent(MouseMovedEvent& e){
        ImGuiIO& io = ImGui::GetIO();
		io.MousePos = ImVec2(e.GetX(), e.GetY());

        return false;
    }

    bool GUI::OnMouseScrollEvent(MouseScrolledEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		io.MouseWheelH += e.GetXOffset();
		io.MouseWheel += e.GetYOffset();

        return false;
    }

    bool GUI::OnKeyPressedEvent(KeyPressedEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		io.KeysDown[(int)e.GetKeyCode()] = true;

		io.KeyCtrl = io.KeysDown[(int)RE_KEY_LEFT_CONTROL] || io.KeysDown[(int)RE_KEY_RIGHT_CONTROL];
		io.KeyShift = io.KeysDown[(int)RE_KEY_LEFT_SHIFT] || io.KeysDown[(int)RE_KEY_RIGHT_SHIFT];
		io.KeyAlt = io.KeysDown[(int)RE_KEY_LEFT_ALT] || io.KeysDown[(int)RE_KEY_RIGHT_ALT];
		io.KeySuper = io.KeysDown[(int)RE_KEY_LEFT_SUPER] || io.KeysDown[(int)RE_KEY_RIGHT_SUPER];

        return false;
    }

    bool GUI::OnKeyReleasedEvent(KeyReleasedEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		io.KeysDown[(int)e.GetKeyCode()] = false;

        return false;
    }

    bool GUI::OnKeyTypedEvent(KeyTypedEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		int keycode = (int)e.GetKeyCode();
		if (keycode > 0 && keycode < 0x10000)
			io.AddInputCharacter((unsigned short)keycode);

		return false;
    }

    bool GUI::OnWindowResizeEvent(WindowResizeEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		io.DisplaySize = ImVec2(e.GetWidth(), e.GetHeight());
		io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

        return false;
    }

	bool GUI::OnCanvasPointerMovedEvent(CanvasPointerMovedEvent& e){
		ImGuiIO& io = ImGui::GetIO();
		io.MousePos = ImVec2(e.GetX(), e.GetY());

        return false;
	}

} // namespace rendering_engin 


