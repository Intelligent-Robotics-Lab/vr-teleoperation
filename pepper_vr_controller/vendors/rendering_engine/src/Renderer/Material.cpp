#include "rendering_engine/Renderer/Material.h"

namespace rendering_engine
{
    Material::Material(const std::shared_ptr<Shader>& shader, const std::shared_ptr<Texture2D>& diffuseTexture)
        :
        m_Shader(shader),
        m_DiffuseTexture(diffuseTexture)
    {

    }

    Material::~Material(){

    }

    void Material::Bind(){
        m_Shader->Bind();
        m_DiffuseTexture->Bind(0);
    }

    void Material::SetShader(const std::shared_ptr<Shader>& shader){
        m_Shader = shader;
    }

    void Material::SetDiffuseTexture(const std::shared_ptr<Texture2D>& diffuseTexture){
        m_DiffuseTexture = diffuseTexture;
    }

} // namespace rendering_engine
