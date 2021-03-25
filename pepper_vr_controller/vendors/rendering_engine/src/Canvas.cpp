#include "rendering_engine/Canvas.h"

#include <vector>
#include <iostream>

#include <GL/glew.h>

#include "rendering_engine/Events/CanvasEvent.h"

namespace rendering_engine
{
    Canvas::Canvas(float width, float height, uint32_t pxlWidth, uint32_t pxlHeight)
        :
        m_Width(width),
        m_Heigth(height),
        m_PxlWidth(pxlWidth),
        m_PxlHeight(pxlHeight),
        m_Transform(glm::mat4(1.0f))
    {
        std::vector<Vertex> vertices;
        vertices.push_back({glm::vec3(0.0f,   0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 1.0f)});
        vertices.push_back({glm::vec3(width,  0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 1.0f)});
        vertices.push_back({glm::vec3(width, -height, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 0.0f)});
        vertices.push_back({glm::vec3(0.0f,  -height, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)});

        std::vector<unsigned int> indices;
        indices.insert(indices.end(), {0, 1, 2});
        indices.insert(indices.end(), {2, 3, 0});

        m_Mesh = std::make_unique<Mesh>(vertices, indices);

        m_FrameBufferSpec = FramebufferSpecification({m_PxlWidth, m_PxlHeight});
        m_Framebuffer = std::make_unique<Framebuffer>(m_FrameBufferSpec);

        m_BufferColorAttachment = m_Framebuffer->GetColorAttachmentID();
    }
    
    Canvas::~Canvas(){

    }
    
    void Canvas::Draw(){
        glBindTexture(GL_TEXTURE_2D, m_BufferColorAttachment);
        m_Mesh->Draw();
    }

    void Canvas::OnUpdate(glm::mat4 deviceTransform){
        glm::mat4 pointerInCanvasFrame = glm::inverse(m_Transform) * deviceTransform;

        glm::vec3 position = pointerInCanvasFrame[3];
        if (position.z <= 0.0f) return;

        glm::vec3 direction = position - glm::vec3(pointerInCanvasFrame * glm::vec4(0.0f, 0.0f, -1.0f, 1.0f));


	    glm::vec3 ans = position - direction * (position.z / direction.z);
        
        if (ans.x <= 0 || ans.x >= m_Width || ans.y <= -m_Heigth || ans.y >= 0) return;

        CanvasPointerMovedEvent event((ans.x/m_Width)*(float)m_PxlWidth, -(ans.y/m_Heigth)*(float)m_PxlHeight);
        m_EventCallback(event);
    }
    
    void Canvas::SetTransform(glm::mat4 transform){
        m_Transform = transform;
    }
    
    glm::mat4 Canvas::GetTransform() const {
        return m_Transform;
    }
    
    float Canvas::GetWidth() const {
        return m_Width;
    }
    
    float Canvas::GetHeight() const {
        return m_Heigth;
    }


} // namespace rendering_engine
