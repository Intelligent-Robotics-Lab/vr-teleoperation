#include "rendering_engine/Renderer/Renderer.h"

#include "GL/glew.h"

namespace rendering_engine{
    
    void Renderer::Init(){
        glewInit();
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
        glEnable(GL_DEPTH_TEST);
    }

    void Renderer::Shutdown(){

    }

    void Renderer::ClearColor(float r, float g, float b, float a){
        glClearColor(r, g, b, a);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void Renderer::SetViewport(uint32_t x, uint32_t y, float width, float height){
        glViewport(x, y, width, height);
    }
}