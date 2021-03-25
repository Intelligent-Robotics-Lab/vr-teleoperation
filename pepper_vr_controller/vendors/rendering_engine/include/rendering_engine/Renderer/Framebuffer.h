#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <stdint.h>

namespace rendering_engine {

struct FramebufferSpecification {
    unsigned int Width, Height;
    // FramebufferFormat Format =
    unsigned int Sample = 1;

    bool SwapChainTarget = false;
};


class Framebuffer {
public:
	Framebuffer(const FramebufferSpecification& spec);
	virtual ~Framebuffer();
    
    void Invalidate();

	void Bind();
	void Unbind();

	void Resize(uint32_t width, uint32_t height);

	uint32_t GetColorAttachmentID() const;

    const FramebufferSpecification& GetSpecification() const;

private:
	uint32_t m_FramebufferID = 0;
	uint32_t m_DepthAttachment = 0; 
	uint32_t m_ColorAttachment = 0;
	FramebufferSpecification m_Specification;

};
}

#endif // FRAMEBUFFER_H