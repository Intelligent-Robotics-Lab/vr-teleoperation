#ifndef VERTEX_H
#define VERTEX_H

#include <glm/glm.hpp>

namespace rendering_engine{

struct Vertex
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texCoords;
//    glm::vec3 tangent;
//    glm::vec3 bitangent;
};

}

#endif // VERTEX_H