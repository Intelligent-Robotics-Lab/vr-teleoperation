#include "rendering_engine/WindowOverlay.h"

namespace rendering_engine {

WindowOverlay::WindowOverlay(uint32_t bufferColorAttachment)
    :
    m_BufferColorAttachment(bufferColorAttachment)
{
    std::vector<Vertex> vertices;
    vertices.push_back({glm::vec3(-1.0f,  1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 1.0f)});
    vertices.push_back({glm::vec3( 1.0f,  1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 1.0f)});
    vertices.push_back({glm::vec3( 1.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 0.0f)});
    vertices.push_back({glm::vec3(-1.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)});

    std::vector<unsigned int> indices;
    indices.insert(indices.end(), {0, 1, 2});
    indices.insert(indices.end(), {2, 3, 0});

    m_Mesh = std::make_unique<Mesh>(vertices, indices);

    m_Shader = std::make_unique<Shader>("/home/alex/ros/src/vr-teleoperation/pepper_vr_controller/vendors/rendering_engine/shaders/WindowOverlay.vert", "/home/alex/ros/src/vr-teleoperation/pepper_vr_controller/vendors/rendering_engine/shaders/WindowOverlay.frag");
}

WindowOverlay::~WindowOverlay()
{
}

void WindowOverlay::Draw(){
    m_Shader->Bind();
    glBindTexture(GL_TEXTURE_2D, m_BufferColorAttachment);
    m_Mesh->Draw();

}

void WindowOverlay::SetBufferColorAttachment(uint32_t bufferColorAttachment){
    m_BufferColorAttachment = bufferColorAttachment;
}

}