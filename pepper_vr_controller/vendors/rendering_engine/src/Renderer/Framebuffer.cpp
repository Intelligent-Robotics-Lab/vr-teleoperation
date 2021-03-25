#include "rendering_engine/Renderer/Framebuffer.h"

#include <iostream>

#include <GL/glew.h>

#include "rendering_engine/Assertions.h"

#include "rendering_engine/GLErrorHandling.h"

namespace rendering_engine{
    
    Framebuffer::Framebuffer(const FramebufferSpecification& spec)
        :
        m_Specification(spec)
    {
        Invalidate();   
    }

	Framebuffer::~Framebuffer(){
        glDeleteFramebuffers(1, &m_FramebufferID);
		glDeleteTextures(1, &m_ColorAttachment);
		glDeleteTextures(1, &m_DepthAttachment);
    }

    void Framebuffer::Invalidate(){

        if (m_FramebufferID) {
			glDeleteFramebuffers(1, &m_FramebufferID);
			glDeleteTextures(1, &m_ColorAttachment);
			glDeleteTextures(1, &m_DepthAttachment);
		}

        GLCall(glCreateFramebuffers(1, &m_FramebufferID));
        GLCall(glBindFramebuffer(GL_FRAMEBUFFER, m_FramebufferID));

        GLCall(glCreateTextures(GL_TEXTURE_2D, 1, &m_ColorAttachment));
        GLCall(glTextureStorage2D(m_ColorAttachment, 1, GL_RGBA8, m_Specification.Width, m_Specification.Height));
        GLCall(glTextureParameteri(m_ColorAttachment, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
        GLCall(glTextureParameteri(m_ColorAttachment, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

        GLCall(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_ColorAttachment, 0));

        GLCall(glCreateTextures(GL_TEXTURE_2D, 1, &m_DepthAttachment));
        GLCall(glTextureStorage2D(m_DepthAttachment, 1, GL_DEPTH24_STENCIL8, m_Specification.Width, m_Specification.Height));

        GLCall(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, m_DepthAttachment, 0));

        RE_ASSERT(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE, "Framebuffer is incomplete!"); 

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

	void Framebuffer::Bind(){
		glBindFramebuffer(GL_FRAMEBUFFER, m_FramebufferID);
		glViewport(0, 0, m_Specification.Width, m_Specification.Height);
    }

	void Framebuffer::Unbind(){
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

	void Framebuffer::Resize(uint32_t width, uint32_t height){
		m_Specification.Width = width;
		m_Specification.Height = height;
		
		Invalidate();
    }

	uint32_t Framebuffer::GetColorAttachmentID() const {
        return m_ColorAttachment;
    }

    const FramebufferSpecification& Framebuffer::GetSpecification() const {
        return m_Specification;
    }

} // namespace rendering_engine