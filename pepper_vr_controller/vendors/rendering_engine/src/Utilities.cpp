#include "rendering_engine/Utilities.h"

namespace rendering_engine{
namespace utilities{

glm::mat4 hmd44_to_mat4(const vr::HmdMatrix44_t &m) {
    return glm::mat4(
            m.m[0][0], m.m[1][0], m.m[2][0], m.m[3][0],
            m.m[0][1], m.m[1][1], m.m[2][1], m.m[3][1],
            m.m[0][2], m.m[1][2], m.m[2][2], m.m[3][2],
            m.m[0][3], m.m[1][3], m.m[2][3], m.m[3][3]);
}

glm::mat4 hmd34_to_mat4(const vr::HmdMatrix34_t &m) {
    return glm::mat4(
            m.m[0][0], m.m[1][0], m.m[2][0], 0.f,
            m.m[0][1], m.m[1][1], m.m[2][1], 0.f,
            m.m[0][2], m.m[1][2], m.m[2][2], 0.f,
            m.m[0][3], m.m[1][3], m.m[2][3], 1.f);
}

} // namespace utilities
} // namespace rendering_engine
