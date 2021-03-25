#ifndef CANVAS_H
#define CANVAS_H

#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "rendering_engine/Mesh.h"
#include "rendering_engine/Renderer/Texture2D.h"
#include "rendering_engine/Renderer/Framebuffer.h"

#include "rendering_engine/Events/Event.h"

namespace rendering_engine
{
    class Canvas {
    public:
        Canvas(float width, float height, uint32_t pxlWidth, uint32_t pxlHeight);
        ~Canvas();

        using EventCallbackFn = std::function<void(Event&)>;
        void SetEventCallback(const EventCallbackFn& callback) { m_EventCallback = callback; }

        void SetTransform(glm::mat4 transform);
        glm::mat4 GetTransform() const;

        float GetWidth() const;
        float GetHeight() const;
        
        void OnUpdate(glm::mat4 deviceTransform);
        void Draw();

        Framebuffer* GetFramebufferPointer() const { return m_Framebuffer.get(); }
    private:
        glm::mat4 m_Transform;
        std::unique_ptr<Mesh> m_Mesh;

        uint32_t m_BufferColorAttachment;
        FramebufferSpecification m_FrameBufferSpec;
        std::unique_ptr<Framebuffer> m_Framebuffer;

        float m_Width;
        float m_Heigth;

        uint32_t m_PxlWidth;
        uint32_t m_PxlHeight;

    private:
        EventCallbackFn m_EventCallback;
    };
} // namespace rendering_engine

#endif // CANVAS_H