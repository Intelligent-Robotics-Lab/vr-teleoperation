#include "pepper_vr_controller/RobotControllerApp.h"


RobotController::RobotController(int argc, char** argv)
     :
    m_VRconnected(false),
    m_ModeOfOperation(ModeOfOperation::Standby),
    m_Argc(argc),
    m_Argv(argv)
{
    s_Instance = this;

    m_WindowWidth = 1280;
    m_WindowHeight = 720;
}

RobotController::~RobotController(){

}

bool RobotController::Init(){
    RosNode::RosInit(m_Argc, m_Argv);
    m_RosNode = std::make_unique<RosNode>();

    m_VRHmd = std::make_unique<rendering_engine::VRHmd>();
    m_VRconnected = m_VRHmd->Init(0.1f, 100.0f);
    if (m_VRconnected){
        m_VRSystem = rendering_engine::VRSystem::GetInstance();
        m_VRSystem->SetEventCallback(std::bind(&Application::OnEvent, this, std::placeholders::_1));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    m_MainWindow = std::make_unique<rendering_engine::Window>("VR_Robot_Controller", m_WindowWidth, m_WindowHeight, false);
    m_MainWindow->SetEventCallback(std::bind(&RobotController::OnEvent, this, std::placeholders::_1));
    m_MainWindow->SetVSync(false);

    rendering_engine::Renderer::Init();
    ////////////////////////////////////////////////////////////////////////////////////////////////

    m_Shader = std::make_shared<rendering_engine::Shader>("/home/alex/ros/src/vr-teleoperation/pepper_vr_controller/vendors/rendering_engine/shaders/MainShader.vert", 
                                                          "/home/alex/ros/src/vr-teleoperation/pepper_vr_controller/vendors/rendering_engine/shaders/MainShader.frag");
    m_Shader->Bind();
    m_Shader->SetUniformFloat("u_AmbientStrength", m_AmbientStrength);
    m_Shader->SetUniformFloat3("u_LightPos", m_MainLightPos);
    m_Shader->SetUniformFloat3("u_LightColor", m_MainLightColor);

    m_Canvas = std::make_unique<rendering_engine::Canvas>(1.28f, 0.72f, m_WindowWidth, m_WindowHeight);
    m_Canvas->SetEventCallback(std::bind(&Application::OnEvent, this, std::placeholders::_1));
    
    m_StandByCanvasTransform = glm::mat4(1.0);
    m_StandByCanvasTransform = glm::translate(m_StandByCanvasTransform, glm::vec3(-0.64f, 2.0f, -0.5f));
    m_StandByCanvasTransform = glm::rotate(m_StandByCanvasTransform, glm::radians(10.0f),glm::vec3(1.0f, 0.0f, 0.0f));
    m_Canvas->SetTransform(m_StandByCanvasTransform);

    m_RobotControlCanvasTransform = glm::mat4(1.0);
    m_RobotControlCanvasTransform = glm::translate(m_RobotControlCanvasTransform, glm::vec3(-0.64f, 1.0f, -0.5f));
    m_RobotControlCanvasTransform = glm::rotate(m_RobotControlCanvasTransform, glm::radians(-30.0f),glm::vec3(1.0f, 0.0f, 0.0f));

    m_WindowOverlay = std::make_unique<rendering_engine::WindowOverlay>(m_Canvas->GetFramebufferPointer()->GetColorAttachmentID());
    
    m_GUI = std::make_unique<rendering_engine::GUI>();
    
    m_CameraController = std::make_unique<rendering_engine::PerspectiveCameraController>(glm::radians(45.0f), (float)m_WindowWidth/(float)m_WindowHeight, 0.1f, 100.0f);

    if (m_VRconnected){
        rendering_engine::VRInput::Init();
        m_ViveController = std::make_unique<rendering_engine::Model>("/home/alex/ros/src/vr-teleoperation/pepper_vr_controller/vendors/rendering_engine/models/vr_controller_vive_1_5/vr_controller_vive_1_5.obj", false, m_Shader);
        //m_ViveController = std::make_unique<rendering_engine::Model>("/home/yomzyo/temp/leftarm.dae", false, m_Shader);

        m_VRWidth = m_VRHmd->GetRecomendedRenderSizeWidth();
        m_VRHeight = m_VRHmd->GetRecomendedRenderSizeHeight();

        m_EyeBufferSpec = rendering_engine::FramebufferSpecification({m_VRWidth, m_VRHeight});
        m_LeftEyeBuffer = std::make_unique<rendering_engine::Framebuffer>(m_EyeBufferSpec);
        m_RightEyeBuffer = std::make_unique<rendering_engine::Framebuffer>(m_EyeBufferSpec);
    }
    
    m_CameraView = std::make_unique<CameraView>(640, 480, m_Shader);
    // m_CameraView = std::make_unique<CameraView>(320, 240, m_Shader);
    

    m_RightControllerTransform = glm::mat4(0.0f);
    m_LeftControllerTransform = glm::mat4(0.0f);
    m_UserReadyToControl = false;
    m_Calibration = false;

    m_RightGripperPrevState = false;
    m_LeftGripperPrevState = false;
    m_GripperRisingEdge = false;
}

bool RobotController::Run(){
    while(m_Running){
        rendering_engine::Timestep currentTime = rendering_engine::Timestep::GetCurrentTime();
        rendering_engine::Timestep timestep = currentTime - m_LastFrameTime;
        m_LastFrameTime = currentTime;

        m_CameraController->OnUpdate(timestep);

        m_Canvas->GetFramebufferPointer()->Bind();
        rendering_engine::Renderer::ClearColor(0.0f, 0.0f, 0.0f, 0.2f);
        rendering_engine::Renderer::SetViewport(0, 0, m_WindowWidth, m_WindowHeight);
        m_GUI->Begin(timestep);
            draw_gui();
        m_GUI->End();
        m_Canvas->GetFramebufferPointer()->Unbind();

        rendering_engine::Renderer::ClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        rendering_engine::Renderer::SetViewport(0, 0, m_WindowWidth, m_WindowHeight);
        m_Shader->Bind();
        m_Shader->SetUniformFloat3("u_ViewPos", m_CameraController->GetCamera().GetPosition());
        m_Shader->SetUniformMat4("u_Projection", m_CameraController->GetCamera().GetProjectionMatrix());
        m_Shader->SetUniformMat4("u_View", m_CameraController->GetCamera().GetViewMatrix());

        m_RosNode->OnUpdate();
        
        if (m_VRconnected){
            rendering_engine::VRInput::OnUpdate();
            m_RightControllerTransform = rendering_engine::VRInput::GetRightHandTransform();
            m_LeftControllerTransform = rendering_engine::VRInput::GetLeftHandTransform();

            m_ViveController->SetTransform(m_LeftControllerTransform);
            m_ViveController->Draw();
            m_ViveController->SetTransform(m_RightControllerTransform);
            m_ViveController->Draw();

            m_Canvas->OnUpdate(m_RightControllerTransform);
            if (m_VRControlOwner && m_ModeOfOperation == ModeOfOperation::Standby) 
                m_GUI->SetClickInputStatus(rendering_engine::VRInput::GetRightTriggerValue() > 0.3 ? true : false);
        }

        m_Shader->SetUniformMat4("u_Model", m_Canvas->GetTransform());
        m_Canvas->Draw();
        m_WindowOverlay->Draw();

        m_GripperRisingEdge = (!m_RightGripperPrevState) & rendering_engine::VRInput::GetRightGripperPressed();
        m_RightGripperPrevState = rendering_engine::VRInput::GetRightGripperPressed();

        switch (m_ModeOfOperation) {
        case ModeOfOperation::Standby:
        {
            if (m_VRconnected && m_GripperRisingEdge){
                m_ModeOfOperation = ModeOfOperation::RobotControl;
                m_Canvas->SetTransform(m_RobotControlCanvasTransform);
                break;
            }
            if (m_VRconnected && m_Calibration)
                m_ModeOfOperation = ModeOfOperation::Calibration;
            break;
        }
        case ModeOfOperation::RobotControl:
        {
            if (m_GripperRisingEdge){
                m_ModeOfOperation = ModeOfOperation::Standby;
                m_UserReadyToControl = false;
                m_Canvas->SetTransform(m_StandByCanvasTransform);
            }

            //m_FakeHeadTrans = glm::mat4(1.0);
            //m_FakeHeadTrans = glm::translate(m_FakeHeadTrans, glm::vec3(m_FakeHeadX, m_FakeHeadY, m_FakeHeadZ));
            //m_FakeHeadTrans = m_FakeHeadTrans * glm::mat4(glm::quat(glm::vec3(m_FakeHeadRotX, m_FakeHeadRotY, m_FakeHeadRotZ)));
            //m_RosNode->SetHead(m_FakeHeadTrans);
            m_RosNode->SetHead(m_VRHmd->GetTransform());

            //glm::mat4 FakeHand = glm::translate(glm::mat4(1.0), glm::vec3(0.21, 1.00, -1.35));
            //FakeHand = glm::rotate(FakeHand, glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            //m_RosNode->SetRightArm(FakeHand);
            m_RosNode->SetRightArm(m_RightControllerTransform);

            m_RosNode->SetLeftArm(m_LeftControllerTransform);
            m_RosNode->SetRightHand(rendering_engine::VRInput::GetRightTriggerValue());
            m_RosNode->SetLeftHand(rendering_engine::VRInput::GetLeftTriggerValue());
                
            glm::mat4 camViewTransform = glm::translate(m_VRHmd->GetTransform(), glm::vec3(0.0f, 0.0f, -3.0));
            m_CameraView->SetTransform(camViewTransform);
            if (!m_RosNode->GetImageData().empty()) {
                m_CameraView->Draw(m_RosNode->GetImageData());
            }
            

            break;
        }
        case ModeOfOperation::Calibration:
        {
            if (m_VRconnected && m_Calibration && m_GripperRisingEdge){
                //m_RosNode->SetHead(m_VRHmd->GetTransform());
                //m_RosNode->CalibrateRightArm(m_RightControllerTransform);
                //m_RosNode->CalibrateLeftArm(m_LeftControllerTransform);
                m_RosNode->SetHeadNew(m_VRHmd->GetTransform());
                m_RosNode->CalibrateRightArm(m_RightControllerTransform);
                m_RosNode->CalibrateLeftArm(m_LeftControllerTransform);
                m_Calibration = false;
                m_ModeOfOperation = ModeOfOperation::Standby;
            }

            if (!m_VRconnected || !m_Calibration)
                m_ModeOfOperation = ModeOfOperation::Standby;

            break;
        }
        default:
            break;
        }
////////////////////////////////////////////////////////////////////////////////////////////////////

        if (m_VRconnected){
            m_VRHmd->OnUpdate();
            rendering_engine::VRInput::OnUpdate();
            m_VRSystem->OnUpdate();

            // Left Eye
            m_LeftEyeBuffer->Bind();

            rendering_engine::Renderer::ClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            rendering_engine::Renderer::SetViewport(0, 0, m_VRWidth, m_VRHeight);
                
            m_Shader->Bind();
            m_Shader->SetUniformFloat3("u_ViewPos", m_VRHmd->GetCamera().GetPositionOfEye(rendering_engine::LeftEye));
            m_Shader->SetUniformMat4("u_Projection", m_VRHmd->GetCamera().GetProjectionMatrix(rendering_engine::LeftEye));
            m_Shader->SetUniformMat4("u_View", m_VRHmd->GetCamera().GetViewMatrix(rendering_engine::LeftEye));
            
            m_ViveController->SetTransform(m_LeftControllerTransform);
            m_ViveController->Draw();
            m_ViveController->SetTransform(m_RightControllerTransform);
            m_ViveController->Draw();

            if (m_ModeOfOperation == ModeOfOperation::RobotControl && !m_RosNode->GetImageData().empty()) 
                m_CameraView->Draw(m_RosNode->GetImageData()); 

            model = m_Canvas->GetTransform();
            m_Shader->SetUniformMat4("u_Model", model);
            m_Canvas->Draw();

            m_LeftEyeBuffer->Unbind();


            // Right Eye
            m_RightEyeBuffer->Bind();

            rendering_engine::Renderer::ClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            rendering_engine::Renderer::SetViewport(0, 0, m_VRWidth, m_VRHeight);

            m_Shader->Bind();
            m_Shader->SetUniformFloat3("u_ViewPos", m_VRHmd->GetCamera().GetPositionOfEye(rendering_engine::RightEye));
            m_Shader->SetUniformMat4("u_Projection", m_VRHmd->GetCamera().GetProjectionMatrix(rendering_engine::RightEye));
            m_Shader->SetUniformMat4("u_View", m_VRHmd->GetCamera().GetViewMatrix(rendering_engine::RightEye));

            m_ViveController->SetTransform(m_LeftControllerTransform);
            m_ViveController->Draw();
            m_ViveController->SetTransform(m_RightControllerTransform);
            m_ViveController->Draw();

            if (m_ModeOfOperation == ModeOfOperation::RobotControl && !m_RosNode->GetImageData().empty()) 
                m_CameraView->Draw(m_RosNode->GetImageData()); 

            model = m_Canvas->GetTransform();
            m_Shader->SetUniformMat4("u_Model", model);
            m_Canvas->Draw();

            m_RightEyeBuffer->Unbind();

            vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)m_LeftEyeBuffer->GetColorAttachmentID(), vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
            vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
            vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)m_RightEyeBuffer->GetColorAttachmentID(), vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
            vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
        }

        m_MainWindow->OnUpdate();
    }
}

void RobotController::draw_gui(){
    ImGui::Begin("VR Robot Controller Dashboard");

    ImGui::Text("Mode of Operation:         "); ImGui::SameLine(); 
    switch (m_ModeOfOperation) {
        case ModeOfOperation::Standby:
            ImGui::Text("Stand by");
            break;
        case ModeOfOperation::RobotControl:
            ImGui::Text("Robot Control");
            break;
        case ModeOfOperation::Calibration:
            ImGui::Text("Calibration");
            break;
        default:
            break;
    }
    
    ImGui::NewLine();
    ImGui::Text("VR System Status:          "); ImGui::SameLine();
    m_VRconnected ? ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Connnected") : ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Disconnected");
     
    ImGui::NewLine();
    ImGui::Text("Right Controller Status:   "); ImGui::SameLine();
    m_RightControllerTransform == glm::mat4(0.0f) ? ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Error") : ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Connnected");
    ImGui::Text("Left Controller Status:    "); ImGui::SameLine();
    m_LeftControllerTransform == glm::mat4(0.0f) ? ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Error") : ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Connnected");

    ImGui::NewLine();
    ImGui::SliderFloat("RShoulderPitch", &m_RosNode->ShoulderPitch, -2.0857, 2.0857);
    ImGui::SliderFloat("RShoulderRoll", &m_RosNode->ShoulderRoll, 0.0087, 1.5620);    
    ImGui::SliderFloat("RElbowYaw", &m_RosNode->ElbowYaw, -2.0857, 2.0857);
    ImGui::SliderFloat("RElbowRoll", &m_RosNode->ElbowRoll, 1.5620, 0.0087);
    ImGui::SliderFloat("RWristYaw", &m_RosNode->WristYaw, -1.8239, 1.8239);

    ImGui::NewLine();
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();

    ImGui::Begin("Robot Controller");
    ImGui::NewLine();
    ImGui::Text("To begin robot control, confirm that you are ready by checking check box below.");
    ImGui::Text("Then press right gripper button to start control.");
    ImGui::NewLine();
    ImGui::Checkbox("User ready", &m_UserReadyToControl);
    ImGui::NewLine();
    ImGui::NewLine();
    ImGui::Text("If you want to calibrate robot's arms, check check box below.");
    ImGui::Text("After that, extend your arms completely to the sides, while pointing palms down.");
    ImGui::Text("Then press right gripper.");
    ImGui::NewLine();
    ImGui::Checkbox("Calibration", &m_Calibration);
    ImGui::End();
}

bool RobotController::Shutdown(){
    if (m_VRconnected)
        rendering_engine::VRSystem::ReleaseInstance();
}

void RobotController::OnEvent(rendering_engine::Event& e){    
    rendering_engine::EventDispatcher dispatcher(e);
    dispatcher.Dispatch<rendering_engine::WindowCloseEvent>(std::bind(&RobotController::OnWindowClose, this, std::placeholders::_1));
    dispatcher.Dispatch<rendering_engine::KeyPressedEvent>(std::bind(&RobotController::OnKeyPressed, this, std::placeholders::_1));

    if (!e.Handled) m_GUI->OnEvent(e);
}


bool RobotController::OnWindowClose(rendering_engine::WindowCloseEvent& e){
    m_Running = false;
    return true;
}

bool RobotController::OnKeyPressed(rendering_engine::KeyPressedEvent& e){
    if(e.GetKeyCode() == RE_KEY_ESCAPE) m_Running = false;
    if(e.GetKeyCode() == RE_KEY_SPACE) m_VRControlOwner = !m_VRControlOwner;
}