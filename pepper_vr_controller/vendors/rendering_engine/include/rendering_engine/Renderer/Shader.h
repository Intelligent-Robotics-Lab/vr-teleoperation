#ifndef SHADER_H
#define SHADER_H

#include <string>

#include <GL/glew.h>
#include <glm/glm.hpp>

namespace rendering_engine{

class Shader {

public:
    Shader(std::string vertLocation, std::string fragLocation);
    virtual ~Shader();
    void Bind();
    void Unbind();

    const std::string& GetName() const;

	void SetUniformInt(const std::string& name, int value);
	void SetUniformIntArray(const std::string& name, int* values, uint32_t count);

	void SetUniformFloat(const std::string& name, float value);
    void SetUniformFloat2(const std::string& name, const glm::vec2& value);
	void SetUniformFloat3(const std::string& name, const glm::vec3& value);
	void SetUniformFloat4(const std::string& name, const glm::vec4& value);
	
    void SetUniformMat3(const std::string& name, const glm::mat3& value);
    void SetUniformMat4(const std::string& name, const glm::mat4& value);

private:
    int CompileShader(unsigned int type, const std::string &source);
    void LinkShaders();
    std::string ReadFile(std::string);

private:
    unsigned int m_ShaderID;
    std::string m_Name;
};

}
#endif // SHADER_H