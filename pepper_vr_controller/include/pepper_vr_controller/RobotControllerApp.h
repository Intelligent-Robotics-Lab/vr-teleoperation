#ifndef RobotControllerAPP_H
#define RobotControllerAPP_H

#include <rendering_engine.h>

#include "pepper_vr_controller/RosNode.h"
#include "pepper_vr_controller/CameraView.h"

class RobotController : public rendering_engine::Application
{
public:
    RobotController(int argc, char** argv);
    virtual ~RobotController();

    virtual bool Init() override;
    virtual bool Run() override;
    virtual bool Shutdown() override;

    virtual void OnEvent(rendering_engine::Event& e) override;

private:
    bool OnWindowClose(rendering_engine::WindowCloseEvent& e);
    bool OnKeyPressed(rendering_engine::KeyPressedEvent& e);

private:
    
private:
    int m_Argc;
    char** m_Argv;

private:
    std::unique_ptr<RosNode> m_RosNode;
    std::unique_ptr<CameraView> m_CameraView;

private:
    rendering_engine::Timestep m_LastFrameTime;

    float m_AmbientStrength = 0.3f;
    glm::vec3 m_MainLightPos = glm::vec3(0.0f, 10.0f, 10.0f);
    glm::vec3 m_MainLightColor = glm::vec3(1.0f, 1.0f, 1.0f);
    float m_SpecularLightStrength = 1.0f;
    glm::vec3 m_ViewPos = glm::vec3(0.0f, 0.0f, 0.0f);

    std::shared_ptr<rendering_engine::Shader> m_Shader;
    std::unique_ptr<rendering_engine::PerspectiveCameraController> m_CameraController;

    std::unique_ptr<rendering_engine::GUI> m_GUI;
    std::unique_ptr<rendering_engine::Canvas> m_Canvas;
    std::unique_ptr<rendering_engine::WindowOverlay> m_WindowOverlay;
    void draw_gui();

    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;


private:
    unsigned int m_VRWidth;
    unsigned int m_VRHeight;

    rendering_engine::FramebufferSpecification m_EyeBufferSpec;
    std::unique_ptr<rendering_engine::Framebuffer> m_LeftEyeBuffer; //FramebufferDesc leftEyeDesc;
    std::unique_ptr<rendering_engine::Framebuffer> m_RightEyeBuffer; //FramebufferDesc rightEyeDesc;

private:
    bool m_VRconnected = false;
    bool m_VRControlOwner = true;
    std::unique_ptr<rendering_engine::VRHmd> m_VRHmd;
    std::unique_ptr<rendering_engine::Model> m_ViveController;
    rendering_engine::VRSystem* m_VRSystem;

private:
    glm::mat4 m_RightControllerTransform;
    glm::mat4 m_LeftControllerTransform;

    bool m_RightGripperPrevState;
    bool m_LeftGripperPrevState;
    bool m_GripperRisingEdge;

private:
    enum class ModeOfOperation {
        Standby,
        RobotControl,
        Calibration
    };

    ModeOfOperation m_ModeOfOperation;
    bool m_UserReadyToControl;
    bool m_Calibration;

    glm::mat4 m_StandByCanvasTransform;
    glm::mat4 m_RobotControlCanvasTransform;
private:
    glm::mat4 m_FakeHeadTrans;
    float m_FakeHeadX = 0.0f, m_FakeHeadY = 1.225f, m_FakeHeadZ = 0.0f;
};

#endif // RobotControllerAPP_H