#include "rendering_engine/Renderer/Texture2D.h"

#include "rendering_engine/Assertions.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

namespace rendering_engine{
    Texture2D::Texture2D(uint32_t width, uint32_t height)
        :
        m_Height(height),
        m_Width(width)
    {
        m_InternalFormat = GL_RGBA8;
		m_DataFormat = GL_RGBA;

        glCreateTextures(GL_TEXTURE_2D, 1, &m_TextureID);
        glTextureStorage2D(m_TextureID, 1, m_InternalFormat, m_Width, m_Height);

		glTextureParameteri(m_TextureID, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTextureParameteri(m_TextureID, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		glTextureParameteri(m_TextureID, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTextureParameteri(m_TextureID, GL_TEXTURE_WRAP_T, GL_REPEAT);
    }

    Texture2D::Texture2D(std::string fileLocation, bool flip)
        :
        m_Path(fileLocation)
    {
        int width, height, channels;
        stbi_set_flip_vertically_on_load(flip);
        unsigned char* data = stbi_load(fileLocation.c_str(), &width, &height, &channels, 0);

        RE_ASSERT(data, "Failed to load texture " + fileLocation);

        m_Width = width;
        m_Height = height;

		GLenum internalFormat = 0, dataFormat = 0;
		if (channels == 4) {
			internalFormat = GL_RGBA8;
			dataFormat = GL_RGBA;
		} else if (channels == 3) {
			internalFormat = GL_RGB8;
			dataFormat = GL_RGB;
		}

        m_InternalFormat = internalFormat;
		m_DataFormat = dataFormat;

        RE_ASSERT(internalFormat & dataFormat, "Format not supported");

        glCreateTextures(GL_TEXTURE_2D, 1, &m_TextureID);
        glTextureStorage2D(m_TextureID, 1, GL_RGBA8, m_Width, m_Height);

		glTextureParameteri(m_TextureID, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTextureParameteri(m_TextureID, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		glTextureParameteri(m_TextureID, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTextureParameteri(m_TextureID, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glTextureSubImage2D(m_TextureID, 0, 0, 0, m_Width, m_Height, dataFormat, GL_UNSIGNED_BYTE, data);

        stbi_image_free(data);
    }

    Texture2D::~Texture2D(){
        glDeleteTextures(1, &m_TextureID);
    }

    void Texture2D::SetData(void* data, uint32_t size){
		uint32_t bpp = m_DataFormat == GL_RGBA ? 4 : 3;
		RE_ASSERT(size == m_Width * m_Height * bpp, "Data must be entire texture!");
		glTextureSubImage2D(m_TextureID, 0, 0, 0, m_Width, m_Height, m_DataFormat, GL_UNSIGNED_BYTE, data);
    }

    void Texture2D::Bind(uint32_t slot) const {
		glBindTextureUnit(slot, m_TextureID);
	}

    uint32_t Texture2D::GetWidth() const{
        return m_Width;
    }

    uint32_t Texture2D::GetHeight() const{
        return m_Height;
    }

    uint32_t Texture2D::GetTextureId() const{
        return m_TextureID;
    }

}


// #include "rendering_engine/Texture2D.h"

// #include "rendering_engine/GLErrorHandling.h"

// #define STB_IMAGE_IMPLEMENTATION
// #include "stb_image.h"

// #include <GL/glew.h>
// #include <iostream>

// namespace rendering_engine{
//     Texture2D::Texture2D()    {
//         GLCall(glGenTextures(1, &m_TextureID));
//     }

//     Texture2D::~Texture2D(){
//         Clear();
//     }

//     bool Texture2D::LoadTexture(std::string fileLocation, bool flip){
//         GLCall(glBindTexture(GL_TEXTURE_2D, m_TextureID));

//         GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));	
//         GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));
//         GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
//         GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

//         stbi_set_flip_vertically_on_load(flip);
//         unsigned char* data = stbi_load(fileLocation.c_str(), &m_Width, &m_Height, &m_NChannels, 0);

//         if (data){
//             if(m_NChannels == 3){
//                 GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_Width, m_Height, 0, GL_RGB, GL_UNSIGNED_BYTE, data));
//             }
//             else if (m_NChannels == 4){
//                 GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_Width, m_Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data));
//             }
//         } else {
//             std::cout << "Failed to load texture at: " << fileLocation << std::endl;
//             return false;
//         }
//         stbi_image_free(data);
        
//         GLCall(glGenerateMipmap(GL_TEXTURE_2D));

//         GLCall(glBindTexture(GL_TEXTURE_2D, 0));

//         return true;
//     }

//     void Texture2D::Use(){
//         GLCall(glActiveTexture(GL_TEXTURE0));
//         GLCall(glBindTexture(GL_TEXTURE_2D, m_TextureID));
//     }

//     void Texture2D::Clear(){
//         GLCall(glDeleteTextures(1, &m_TextureID));
//         m_TextureID = 0;
//         m_Width = 0;
//         m_Height = 0;
//         m_NChannels = 0;
//     }

//     void Texture2D::SetTexture(unsigned int textureID, int width, int height, int numChannels){
//         Clear();
//         m_TextureID = textureID;
//         m_Width = width;
//         m_Height = height;
//         m_NChannels = numChannels;
//     }

// }