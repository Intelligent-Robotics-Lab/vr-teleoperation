#ifndef CANVAS_EVENT_H
#define CANVAS_EVENT_H

#include "rendering_engine/Events/Event.h"

namespace rendering_engine {
    
    class CanvasPointerMovedEvent : public Event
    {
    public:
        CanvasPointerMovedEvent(float x, float y)
            : m_X(x), m_Y(y) {}

        float GetX() const { return m_X; }
        float GetY() const { return m_Y; }

        std::string ToString() const override
        {
            std::stringstream ss;
            ss << "CanvasPointerMovedEvent: " << m_X << ", " << m_Y;
            return ss.str();
        }
        
        EVENT_CLASS_TYPE(CanvasPointerMoved)
        EVENT_CLASS_CATEGORY(EventCategoryInput | EventCategoryCanvasPointer)
    private:
        float m_X, m_Y;
    };
    
} // namespace rendering_engine


#endif // CANVAS_EVENT_H