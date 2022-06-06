#include "rendering_engine/Model.h"

#include <iostream>
#include <ros/package.h>

namespace rendering_engine{

Model::Model(std::string filelocation, bool flipTextures, std::shared_ptr<Shader>& shader)
    : IDrawable(), m_Shader(shader)
{
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(filelocation, aiProcess_Triangulate | aiProcess_FlipUVs);
    
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode){
        std::cout << "Assimp couldn't load model: " << importer.GetErrorString() << std::endl;
        return;
    }
    
    processNode(scene->mRootNode, scene);
    processMaterials(scene, flipTextures, shader);
}

Model::~Model(){

}

void Model::Draw(){
    for (unsigned int i = 0; i < m_Meshes.size(); i++){

        unsigned int materialIndex = m_MeshToMaterial[i];

        if(materialIndex < m_Materials.size() && m_Materials[materialIndex]){
            m_Materials[materialIndex]->Bind();
        }

        m_Shader->SetUniformMat4("u_Model", m_Transform);

        m_Meshes[i]->Draw();
    }
}

const std::vector<std::shared_ptr<Material>>& Model::GetMaterials() const {
    return m_Materials;
}

const std::vector<std::shared_ptr<Mesh>>& Model::GetMeshes() const {
    return m_Meshes;
}

const std::vector<unsigned int>& Model::GetMeshToMaterial() const {
    return m_MeshToMaterial;
}

const glm::mat4& Model::GetTransform() const {
    return m_Transform;
}

void Model::SetTransform(const glm::mat4& transform){
    m_Transform = transform;
}

void Model::processNode(aiNode *node, const aiScene *scene){
    // process all the node's meshes
    for(unsigned int i = 0; i < node->mNumMeshes; i++){
        processMesh(scene->mMeshes[node->mMeshes[i]], scene);
    }
    // do the same for each child
    for(unsigned int i = 0; i < node->mNumChildren; i++){
        processNode(node->mChildren[i], scene);
    }
}

void Model::processMesh(aiMesh *mesh, const aiScene *scene){
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    // Process Vertices
    for(unsigned int i = 0; i < mesh->mNumVertices; i++){
        Vertex vertex;
        
        vertex.position.x = mesh->mVertices[i].x;
        vertex.position.y = mesh->mVertices[i].y;
        vertex.position.z = mesh->mVertices[i].z;

        vertex.normal.x = mesh->mNormals[i].x;
        vertex.normal.y = mesh->mNormals[i].y;
        vertex.normal.z = mesh->mNormals[i].z;

        if(mesh->mTextureCoords[0]){
            vertex.texCoords.x = mesh->mTextureCoords[0][i].x;
            vertex.texCoords.y = mesh->mTextureCoords[0][i].y;
        } else {
            vertex.texCoords = glm::vec2(0.0f, 0.0f);
        }
        vertices.push_back(vertex);
    }

    // Process Indices
    for(unsigned int i = 0; i < mesh->mNumFaces; i++){
        aiFace face = mesh->mFaces[i];
        for(unsigned int j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);
    }

    std::shared_ptr<Mesh> newMesh = std::make_shared<Mesh>(vertices, indices);
    m_Meshes.push_back(newMesh);
    m_MeshToMaterial.push_back(mesh->mMaterialIndex);
}

void Model::processMaterials(const aiScene *scene, bool flip, std::shared_ptr<Shader>& shader){

    m_Materials.resize(scene->mNumMaterials);
	
	for (unsigned int i = 0; i < scene->mNumMaterials; i++) {
		aiMaterial* material = scene->mMaterials[i];
		m_Materials[i] = nullptr;
		if (material->GetTextureCount(aiTextureType_DIFFUSE)) {
			aiString path;
			if (material->GetTexture(aiTextureType_DIFFUSE, 0, &path) == AI_SUCCESS) {
				int idx = std::string(path.data).rfind("\\");
				std::string filename = std::string(path.data).substr(idx + 1);

				std::string texPath = std::string(ros::package::getPath("pepper_vr_controller") + "/vendors/rendering_engine/textures/") + filename;

                std::shared_ptr<Texture2D> texture = std::make_shared<Texture2D>(texPath, flip);
				m_Materials[i] = std::make_shared<Material>(shader, texture);
			}
		}
	}
}


}