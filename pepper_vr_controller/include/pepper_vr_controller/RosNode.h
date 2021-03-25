#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

#include <rendering_engine.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

class RosNode {
public:
    static void RosInit(int argc, char** argv);

    RosNode();
    virtual ~RosNode();

    void OnUpdate();

    cv::Mat& GetImageData();
    int GetImageWidth();
    int GetImageHeight();

    void SetHead(glm::mat4 transform);
    void SetLeftArm(glm::mat4 transform);
    void SetRightArm(glm::mat4 transform);
    void SetRightHand(float gripperVal);
    void SetLeftHand(float gripperVal);

    void SetHeadNew(glm::mat4 transform);
    void SetLeftArmNew(glm::mat4 transform);
    void SetRightArmNew(glm::mat4 transform);
    void CalibrateRightArmNew(glm::mat4 transform);
    void CalibrateLeftArmNew(glm::mat4 transform);

    void CalibrateRightArm(glm::mat4 transform);
    void CalibrateLeftArm(glm::mat4 transform);

    // Temporary
    float ElbowRoll, ElbowYaw, ShoulderPitch, ShoulderRoll, WristYaw;

private:
    ros::NodeHandle m_Node;
    std::unique_ptr<ros::Publisher> m_JointAnglesPub;
    naoqi_bridge_msgs::JointAnglesWithSpeed m_JointAnglesMsg;
    ros::Timer m_Timer;
private:
    glm::mat4 m_GroundToHeadTransform;
    glm::mat4 m_GroundToRightControllerTransform;
    glm::mat4 m_GroundToLeftControllerTransform;

    glm::vec4 p_r_elbow_hand;
    glm::vec3 e_r_forearm_hand;
    glm::vec3 e_r_palm_hand;
    glm::vec4 p_l_elbow_hand;
    glm::vec3 e_l_forearm_hand;
    glm::vec3 e_l_palm_hand;

private:
    cv::Mat m_Image;
	cv_bridge::CvImagePtr m_CVImagePtr;
	image_transport::ImageTransport m_ImageTransport;
	std::unique_ptr<image_transport::Subscriber> m_ImageSub;

private:
	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void TimerCallback(const ros::TimerEvent&);

    void PublishTransform(glm::mat4 transform, std::string parent, std::string child);
    tf::TransformBroadcaster m_TransformBroadcaster;

    void PublishTransformMsg(glm::mat4 transform, std::string parent, std::string child);
    std::unique_ptr<ros::Publisher> m_TransformMsgPub;

private:
    glm::quat RotationBetweenVectors(glm::vec3 start, glm::vec3 dest);
    glm::quat LookAt(glm::vec3 direction, glm::vec3 desiredUp);
    
private:
    enum AngleIndex {
        HeadPitch, HeadYaw,
        LElbowRoll, LElbowYaw, LShoulderPitch, LShoulderRoll, LWristYaw,
        RElbowRoll, RElbowYaw, RShoulderPitch, RShoulderRoll, RWristYaw,
        RHand, LHand
    };
};

#endif // ROS_NODE_H