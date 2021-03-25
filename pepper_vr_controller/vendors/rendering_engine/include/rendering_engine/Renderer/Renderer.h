#ifndef RENDERER_H
#define RENDERER_H

#include <memory>

#include <glm/glm.hpp>

namespace rendering_engine{


class Renderer {
public:
    static void Init();
    static void Shutdown();

    static void ClearColor(float r, float g, float b, float a);
    static void SetViewport(uint32_t x, uint32_t y, float width, float height);
};
    
} // namespace rendering_engine


#endif // RENDERER_H