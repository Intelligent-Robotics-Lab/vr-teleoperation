#ifndef UTILITIES_H
#define UTILITIES_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <openvr.h>

namespace rendering_engine{
namespace utilities{
    glm::mat4 hmd44_to_mat4(const vr::HmdMatrix44_t &m);
    glm::mat4 hmd34_to_mat4(const vr::HmdMatrix34_t &m);
}
}

#endif // UTILITIES_H