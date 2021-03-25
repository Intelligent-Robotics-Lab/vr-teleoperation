#ifndef GUI_H
#define GUI_H

#include "rendering_engine/Events/ApplicationEvent.h"
#include "rendering_engine/Events/KeyEvent.h"
#include "rendering_engine/Events/MouseEvent.h"
#include "rendering_engine/Events/CanvasEvent.h"

#include "rendering_engine/Timestep.h"

namespace rendering_engine{

class GUI {
public:
    GUI();
    ~GUI();

    void OnEvent(Event& event);

    void Begin(Timestep ts);
    void End();

    void SetClickInputStatus(bool click);

private:
    bool OnMouseButtonPressedEvent(MouseButtonPressedEvent& e);
    bool OnMouseButtonReleaseEvent(MouseButtonReleasedEvent& e);
    bool OnMouseMovedEvent(MouseMovedEvent& e);
    bool OnMouseScrollEvent(MouseScrolledEvent& e);
    bool OnKeyPressedEvent(KeyPressedEvent& e);
    bool OnKeyReleasedEvent(KeyReleasedEvent& e);
    bool OnKeyTypedEvent(KeyTypedEvent& e);
    bool OnWindowResizeEvent(WindowResizeEvent& e);

    bool OnCanvasPointerMovedEvent(CanvasPointerMovedEvent& e);
};

} // namespace rendering_engine

#endif // GUI_H
