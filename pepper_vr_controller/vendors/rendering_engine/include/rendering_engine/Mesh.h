#ifndef MESH_H
#define MESH_H

#include <vector>

#include "rendering_engine/Vertex.h"

namespace rendering_engine{

class Mesh{
public:
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices);
    ~Mesh();

    void Draw();

private:
    bool Init();

private:
    std::vector<Vertex> m_Vertices;
    std::vector<unsigned int> m_Indices;

    unsigned int m_VAO, m_VBO, m_EBO;
};

}

#endif // MESH_H