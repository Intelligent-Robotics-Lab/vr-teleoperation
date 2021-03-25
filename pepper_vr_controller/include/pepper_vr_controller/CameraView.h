#ifndef CAMERA_VIEW_H
#define CAMERA_VIEW_H

#include <opencv2/opencv.hpp>

#include <rendering_engine.h>

class CameraView{
public:
    CameraView(unsigned int widthPxl, unsigned int heightPxl, std::shared_ptr<rendering_engine::Shader> shader);
    ~CameraView();

    void SetTransform(glm::mat4 transform);

    void Draw(cv::Mat image);

private:
    std::unique_ptr<rendering_engine::Mesh> m_Mesh;
    std::shared_ptr<rendering_engine::Shader> m_Shader;
    std::shared_ptr<rendering_engine::Texture2D> m_Texture;

private:
    glm::mat4 m_Transform;

    unsigned int m_WidthPxl;
    unsigned int m_HeightPxl;
};

#endif // CAMERA_VIEW_H