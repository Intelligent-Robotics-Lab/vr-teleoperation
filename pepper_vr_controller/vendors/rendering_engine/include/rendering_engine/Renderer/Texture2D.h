#ifndef TEXTURE_2D_H
#define TEXTURE_2D_H

// This class was taken from the Hazel engine.
// https://github.com/TheCherno/Hazel/blob/master/Hazel/src/Platform/OpenGL/OpenGLTexture.h

#include <string>
#include <GL/glew.h>

namespace rendering_engine{

class Texture2D{
public:
    Texture2D(uint32_t width, uint32_t height);
    Texture2D(std::string fileLocation, bool flip);
    virtual ~Texture2D();

    uint32_t GetWidth() const;
    uint32_t GetHeight() const;
    uint32_t GetTextureId() const;

    void SetData(void* data, uint32_t size);

    void Bind(uint32_t slot = 0) const;

    bool operator==(const Texture2D& other) const {
		return m_TextureID == other.m_TextureID;
	}

private:
    std::string m_Path;
    uint32_t m_TextureID;
    uint32_t m_Width, m_Height, m_NChannels;
    GLenum m_InternalFormat, m_DataFormat;


};

}

#endif // TEXTURE_2D_H

 
// #ifndef TEXTURE_2D_H
// #define TEXTURE_2D_H

// #include <string>

// namespace rendering_engine{

// class Texture2D{
// public:
//     Texture2D();
//     ~Texture2D();

//     bool LoadTexture(std::string fileLocation, bool flip);
//     void SetTexture(unsigned int textureID, int width, int height, int numChannels);

//     void Use();
//     void Clear();

// private:
//     unsigned int m_TextureID;
//     int m_Width, m_Height, m_NChannels;

// };

// }

// #endif // TEXTURE_2D_H