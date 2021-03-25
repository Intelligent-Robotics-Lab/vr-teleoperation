#include "rendering_engine/ReferenceFrame.h"

#include <vector>

#include <GL/glew.h>
#include "rendering_engine/GLErrorHandling.h"

namespace rendering_engine{
    ReferenceFrame::ReferenceFrame()
        :
        m_Transform(glm::mat4(1.0f)),
        m_VAO(0),
        m_VBO(0),
        m_EBO(0)
    {
        GLCall(glGenVertexArrays(1, &m_VAO));
        GLCall(glGenBuffers(1, &m_VBO));
        GLCall(glGenBuffers(1, &m_EBO));

        GLCall(glBindVertexArray(m_VAO));
        

        std::vector<glm::vec3> vertices;
            vertices.push_back({0.0f, 0.0f, 0.0f});
            vertices.push_back({1.0f, 0.0f, 0.0f});
            vertices.push_back({0.0f, 1.0f, 0.0f});
            vertices.push_back({0.0f, 0.0f, 1.0f});
        GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_VBO));
        GLCall(glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW));

        std::vector<unsigned int> indices;
            indices.insert(indices.end(), {0, 1});
            indices.insert(indices.end(), {0, 2});
            indices.insert(indices.end(), {0, 3});
        GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO));
        GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW));

        GLCall(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0));
        GLCall(glEnableVertexAttribArray(0));
    }

    ReferenceFrame::~ReferenceFrame(){

    }

    void ReferenceFrame::SetTransform(glm::mat4 transform){
        m_Transform = transform;
    }

    void ReferenceFrame::Draw(){
        GLCall(glBindVertexArray(m_VAO));
        GLCall(glLineWidth(5.0f));
        GLCall(glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0));
        GLCall(glBindVertexArray(0));
    }

} // namespace rendering_engine
