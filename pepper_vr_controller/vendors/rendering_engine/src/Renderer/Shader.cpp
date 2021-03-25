#include "rendering_engine/Renderer/Shader.h"

#include <fstream>
#include <iostream>

#include <glm/gtc/type_ptr.hpp>

#include "rendering_engine/Assertions.h"

namespace rendering_engine{

    Shader::Shader(std::string vertLocation, std::string fragLocation){
        std::string vertexSource = ReadFile(vertLocation);
        std::string fragSource = ReadFile(fragLocation);

        unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexSource);
        unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragSource);

        m_ShaderID = glCreateProgram();

        glAttachShader(m_ShaderID, vs);
        glAttachShader(m_ShaderID, fs);

        LinkShaders();

        glDeleteShader(vs);
        glDeleteShader(fs);

        // Extract name from filepath
		auto lastSlash = vertLocation.find_last_of("/\\");
		lastSlash = lastSlash == std::string::npos ? 0 : lastSlash + 1;
		auto lastDot = vertLocation.rfind('.');
		auto count = lastDot == std::string::npos ? vertLocation.size() - lastSlash : lastDot - lastSlash;
		m_Name = vertLocation.substr(lastSlash, count);
    }

    Shader::~Shader(){
		glDeleteProgram(m_ShaderID);
    }
    
    void Shader::Bind(){
        glUseProgram(m_ShaderID);
    }

    void Shader::Unbind(){
        glUseProgram(0);
    }

    const std::string& Shader::GetName() const{
        return m_Name;
    }

	void Shader::SetUniformInt(const std::string& name, int value){
        int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniform1i(location, value);
    }

	void Shader::SetUniformIntArray(const std::string& name, int* values, uint32_t count){
		int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniform1iv(location, count, values);
    }

	void Shader::SetUniformFloat(const std::string& name, float value){
        int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniform1f(location, value);
    }

    void Shader::SetUniformFloat2(const std::string& name, const glm::vec2& value){
        int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniform2f(location, value.x, value.y);
    }

	void Shader::SetUniformFloat3(const std::string& name, const glm::vec3& value){
        int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniform3f(location, value.x, value.y, value.z);
    }

	void Shader::SetUniformFloat4(const std::string& name, const glm::vec4& value){
        int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniform4f(location, value.x, value.y, value.z, value.w);
    }
	
    void Shader::SetUniformMat3(const std::string& name, const glm::mat3& value){
        int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniformMatrix3fv(location, 1, GL_FALSE, glm::value_ptr(value));
    }

    void Shader::SetUniformMat4(const std::string& name, const glm::mat4& value){
        int location = glGetUniformLocation(m_ShaderID, name.c_str());
		glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
    }

    int Shader::CompileShader(unsigned int type, const std::string &source){
        unsigned int id = glCreateShader(type);
        const char* src = source.c_str();
        glShaderSource(id, 1, &src, nullptr);
        glCompileShader(id);
        
        int result;
        glGetShaderiv(id, GL_COMPILE_STATUS, &result);
        if (result == GL_FALSE){
            int length;
            glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
            char* message = (char*)alloca(length * sizeof(char));
            glGetShaderInfoLog(id, length, &length, message);
            std::cout << "Failed to compile " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader" << std::endl;
            std::cout << message << std::endl;
            glDeleteShader(id);
            RE_ASSERT(false, "Shader compilation failure!");
            return 0;
        }
        return id;
    }

    void Shader::LinkShaders(){
        glLinkProgram(m_ShaderID);

        int result;
        glGetProgramiv(m_ShaderID, GL_LINK_STATUS, &result);
        if (!result)
        {
            int length;
            glGetShaderiv(m_ShaderID, GL_INFO_LOG_LENGTH, &length);
            char* message = (char*)alloca(length * sizeof(char));
            glGetProgramInfoLog(m_ShaderID, length, &length, message);
            std::cout << "Failed to link shader" << std::endl;
            std::cout << message << std::endl;
            RE_ASSERT(false, "Shader link failure!");
            return;
        }
    }

    std::string Shader::ReadFile(std::string fileLocation){
        std::string content;
        std::ifstream fileStream(fileLocation, std::ios::in);

        if (!fileStream.is_open()) {
            std::cout << "Failed to read '"<< fileLocation << "'. File doesn't exist." << std::endl;
            return "";
        }

        std::string line = "";
        while (!fileStream.eof())
        {
            std::getline(fileStream, line);
            content.append(line + "\n");
        }

        fileStream.close();
        return content;
    }

}