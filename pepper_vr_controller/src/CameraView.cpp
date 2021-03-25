#include "pepper_vr_controller/CameraView.h"

CameraView::CameraView(unsigned int widthPxl, unsigned int heightPxl, std::shared_ptr<rendering_engine::Shader> shader)
    :
    m_WidthPxl(widthPxl),
    m_HeightPxl(heightPxl),
    m_Shader(shader)
{
    float width = (float)m_WidthPxl/200.0f;
    float height = (float)m_HeightPxl/200.0f;
    std::vector<rendering_engine::Vertex> vertices;
    vertices.push_back({glm::vec3(-width/2.0f, -height/2.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 1.0f)});
    vertices.push_back({glm::vec3( width/2.0f, -height/2.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 1.0f)});
    vertices.push_back({glm::vec3( width/2.0f,  height/2.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 0.0f)});
    vertices.push_back({glm::vec3(-width/2.0f,  height/2.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)});

    std::vector<unsigned int> indices;
    indices.insert(indices.end(), {0, 1, 2});
    indices.insert(indices.end(), {2, 3, 0});

    m_Mesh = std::make_unique<rendering_engine::Mesh>(vertices, indices);

    m_Texture = std::make_shared<rendering_engine::Texture2D>(m_WidthPxl, m_HeightPxl);

    m_Transform = glm::mat4(1.0f);
}

CameraView::~CameraView(){

}

void CameraView::Draw(cv::Mat image){
    m_Texture->Bind();
    m_Texture->SetData(image.ptr(), m_WidthPxl*m_HeightPxl*4);

    m_Shader->Bind();
    m_Shader->SetUniformMat4("u_Model", m_Transform);

    m_Mesh->Draw();
}
    
void CameraView::SetTransform(glm::mat4 transform){
    m_Transform = transform;
}
