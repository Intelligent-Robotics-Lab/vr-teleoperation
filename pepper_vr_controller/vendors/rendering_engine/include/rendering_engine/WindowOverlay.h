#ifndef WINDOW_OVERLAY_H
#define WINDOW_OVERLAY_H

#include <memory>

#include "rendering_engine/Mesh.h"
#include "rendering_engine/Renderer/Shader.h"

namespace rendering_engine {

class WindowOverlay {
public:
    WindowOverlay(uint32_t bufferColorAttachment);
    ~WindowOverlay();

    void Draw();

    void SetBufferColorAttachment(uint32_t bufferColorAttachment);
    
private:
    std::unique_ptr<Mesh> m_Mesh;
    std::unique_ptr<Shader> m_Shader;
    uint32_t m_BufferColorAttachment;
};

}

#endif // WINDOW_OVERLAY_H