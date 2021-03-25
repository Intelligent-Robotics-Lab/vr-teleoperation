#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "rendering_engine/Renderer/IDrawable.h"

namespace rendering_engine{

class Model : public IDrawable{
public:
    Model(std::string filelocation, bool flipTextures, std::shared_ptr<Shader>& shader);
    ~Model();

    void Draw();
    virtual const std::vector<std::shared_ptr<Material>>& GetMaterials() const override;
    virtual const std::vector<std::shared_ptr<Mesh>>& GetMeshes() const override;
    virtual const std::vector<unsigned int>& GetMeshToMaterial() const override;
    virtual const glm::mat4& GetTransform() const override;

    void SetTransform(const glm::mat4& transform);

private:
    std::vector<std::shared_ptr<Material>> m_Materials;
    std::vector<std::shared_ptr<Mesh>> m_Meshes;
    std::vector<unsigned int> m_MeshToMaterial;

    glm::mat4 m_Transform;
    std::shared_ptr<Shader>& m_Shader;

private:
    void processNode(aiNode *node, const aiScene *scene);
    void processMesh(aiMesh *mesh, const aiScene *scene);
    void processMaterials(const aiScene *scene, bool flip, std::shared_ptr<Shader>& shader);

};

} // namesmpace rendering_engine

#endif // MODEL_H