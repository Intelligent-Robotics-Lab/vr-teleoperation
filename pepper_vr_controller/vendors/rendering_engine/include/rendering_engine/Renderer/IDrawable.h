#ifndef I_DRAWABLE_H
#define I_DRAWABLE_H

#include <vector>
#include <memory>

#include <glm/glm.hpp>

#include "rendering_engine/Mesh.h"
#include "rendering_engine/Renderer/Material.h"

namespace rendering_engine{

class IDrawable {
    public:
        virtual const std::vector<std::shared_ptr<Material>>& GetMaterials() const = 0;
        virtual const std::vector<std::shared_ptr<Mesh>>& GetMeshes() const = 0;
        virtual const std::vector<unsigned int>& GetMeshToMaterial() const = 0;
        virtual const glm::mat4& GetTransform() const = 0;
};

} // namespace rendering_engine

#endif // I_DRAWABLE_H