#ifndef REFERENCE_FRAME_H
#define REFERENCE_FRAME_H

#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace rendering_engine
{

class ReferenceFrame{
public:
    ReferenceFrame();
    ~ReferenceFrame();

    void SetTransform(glm::mat4 transform);
    void Draw();
private:
    glm::mat4 m_Transform;

    unsigned int m_VAO, m_VBO, m_EBO;
};

} // namespace rendering_engine

#endif // REFERENCE_FRAME_H