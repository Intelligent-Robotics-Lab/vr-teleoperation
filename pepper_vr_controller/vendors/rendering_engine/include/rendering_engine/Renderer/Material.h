#ifndef MATERIAL_H
#define MATERIAL_H

#include <memory>
#include <vector>

#include "rendering_engine/Renderer/Shader.h"
#include "rendering_engine/Renderer/Texture2D.h"

namespace rendering_engine {
    
    class Material {
        public:
            Material(const std::shared_ptr<Shader>& shader, const std::shared_ptr<Texture2D>& diffuseTexture);
            ~Material();

            void SetShader(const std::shared_ptr<Shader>& shader);

            void SetDiffuseTexture(const std::shared_ptr<Texture2D>& diffuseTexture);

            void Bind();

        private:
            std::shared_ptr<Shader> m_Shader;
            std::shared_ptr<Texture2D> m_DiffuseTexture;
    };

} // namespace rendering_engine


#endif // MATERIAL_H