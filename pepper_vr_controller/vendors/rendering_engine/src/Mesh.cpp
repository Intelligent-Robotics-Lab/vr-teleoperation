#include "rendering_engine/Mesh.h"

#include <iostream>

#include <GL/glew.h>

#include "rendering_engine/GLErrorHandling.h"

namespace rendering_engine{

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices)
    :
    m_Vertices(vertices),
    m_Indices(indices)
{
    if (!Init())
        std::cout << "Could not initialize mesh" << std::endl;
}

Mesh::~Mesh(){

}

bool Mesh::Init(){
    GLCall(glGenVertexArrays(1, &m_VAO));
    GLCall(glGenBuffers(1, &m_VBO));
    GLCall(glGenBuffers(1, &m_EBO));

    GLCall(glBindVertexArray(m_VAO));

    GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_VBO));
    GLCall(glBufferData(GL_ARRAY_BUFFER, m_Vertices.size()*sizeof(Vertex), &m_Vertices[0], GL_STATIC_DRAW));

    GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO));
    GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_Indices.size() * sizeof(unsigned int), &m_Indices[0], GL_STATIC_DRAW));

    // Seting vertex attrib pointers
    // Vertex position
    GLCall(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0));
    GLCall(glEnableVertexAttribArray(0));
    // Vertex normals
    GLCall(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal)));
    GLCall(glEnableVertexAttribArray(1));
    // Vertex texture coordinates
    GLCall(glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords)));
    GLCall(glEnableVertexAttribArray(2));
    // Vertex tangents
    // GLCall(glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tangent)));
    // GLCall(glEnableVertexAttribArray(0));
    // Vertex bitangents
    // GLCall(glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, bitangent)));
    // GLCall(glEnableVertexAttribArray(0));

    GLCall(glBindVertexArray(0));
    return true;
}

void Mesh::Draw(){
    GLCall(glBindVertexArray(m_VAO));
    glDrawElements(GL_TRIANGLES, m_Indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

}