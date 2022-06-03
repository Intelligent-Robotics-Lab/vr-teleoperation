#include "pepper_vr_controller/RosNode.h"

//#include "trajectory_msgs/JointTrajectory.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/vector_angle.hpp>

void RosNode::RosInit(int argc, char** argv){
    std::cout << "RosNode created" << std::endl;
    ros::init(argc, argv, "pepper_vr_controller");
}

RosNode::RosNode()
    :
    m_ImageTransport(m_Node)
{
    p_r_elbow_hand = glm::vec4(0.0, 0.0, 0.35, 1.0);
    e_r_palm_hand = glm::vec3(-1.0, 0.0, 0.0);
    e_r_forearm_hand = glm::vec3(0.0, 0.0, -1.0);
    p_l_elbow_hand = glm::vec4(0.0, 0.0, 0.35, 1.0);
    e_l_forearm_hand = glm::vec3(0.0, 0.0, -1.0);
    e_l_palm_hand = glm::vec3(1.0, 0.0, 0.0);

    m_ImageSub = std::make_unique<image_transport::Subscriber>(m_ImageTransport.subscribe("/camera/image_raw", 1, &RosNode::ImageCallback, this));

    m_JointAnglesPub = std::make_unique<ros::Publisher>(m_Node.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1));
    m_JointAnglesMsg.joint_names.resize(14);
    m_JointAnglesMsg.joint_names = {"HeadPitch", "HeadYaw",
                                    "LElbowRoll", "LElbowYaw", "LShoulderPitch", "LShoulderRoll", "LWristYaw",
                                    "RElbowRoll", "RElbowYaw", "RShoulderPitch", "RShoulderRoll", "RWristYaw",
                                    "RHand", "LHand"};
    m_JointAnglesMsg.joint_angles.resize(14);
    fill(m_JointAnglesMsg.joint_angles.begin(), m_JointAnglesMsg.joint_angles.end(), 0.0f);

    m_Timer = m_Node.createTimer(ros::Duration(0.1), &RosNode::TimerCallback, this);

    m_TransformMsgPub = std::make_unique<ros::Publisher>(m_Node.advertise<geometry_msgs::TransformStamped>("/VRTransforms", 1));


    ///////new code for dpad movement 5/2022
    m_DpadPub = std::make_unique<ros::Publisher>(m_Node.advertise<geometry_msgs::Twist>("/DpadMovement", 1 ));
    
    

}

RosNode::~RosNode(){
    ros::shutdown();
}

void RosNode::TimerCallback(const ros::TimerEvent& event){
    m_JointAnglesMsg.header.stamp = event.current_real;
    m_JointAnglesMsg.speed = 1.0f;
    m_JointAnglesPub->publish(m_JointAnglesMsg);
    
    //new code for dpad movement 5/2022
    if(up) {

        m_TwistMsg.linear.x = 0.5; 
         

    }
    else if(down) { 

        m_TwistMsg.linear.x = -05;
        

    }
    else {

         m_TwistMsg.linear.x = 0;

    }
             
    if(right) {

    m_TwistMsg.angular.z = -0.5; 
    

    }
    else if(left) { 

        m_TwistMsg.angular.z = 0.5;
    }
    else {

         m_TwistMsg.angular.z = 0;

    }
       
    m_DpadPub->publish(m_TwistMsg);
}

void RosNode::ImageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        m_CVImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    m_Image = m_CVImagePtr->image;
}

cv::Mat& RosNode::GetImageData(){
    return m_Image;
}

int RosNode::GetImageWidth(){
    return m_Image.cols;
}

int RosNode::GetImageHeight(){
    return m_Image.rows;
}

void RosNode::SetHead(glm::mat4 transform){
    if (transform != glm::mat4(1.0) || transform != glm::mat4(0.0))
        m_GroundToHeadTransform = transform;

    PublishTransformMsg(transform, "Ground", "HMD");

    glm::quat q(m_GroundToHeadTransform);
    
    m_JointAnglesMsg.joint_angles[AngleIndex::HeadPitch] = -glm::pitch(q);
    m_JointAnglesMsg.joint_angles[AngleIndex::HeadYaw] = glm::yaw(q);
}


void RosNode::SetLeftArm(glm::mat4 transform){
    m_GroundToLeftControllerTransform = transform;
    PublishTransformMsg(transform, "Ground", "LeftController");

    double elbowRoll, elbowYaw, shoulderPitch, shoulderRoll, wristYaw;

    glm::mat4 t_shoulder_to_head = { 0.0,  0.0, -1.0,  0.000,
                                    -1.0,  0.0,  0.0, -0.210,
                                     0.0,  1.0,  0.0,  0.225,
                                     0.0,  0.0,  0.0,  1.000};
    t_shoulder_to_head = glm::transpose(t_shoulder_to_head) * glm::mat4(glm::mat3(m_GroundToHeadTransform));
    glm::mat4 t_shoulder_to_hand = t_shoulder_to_head * glm::inverse(m_GroundToHeadTransform) * transform;
    glm::vec4 p_elbow_shoulder = t_shoulder_to_hand * p_l_elbow_hand;
    if (p_elbow_shoulder.x < 0.01 ) p_elbow_shoulder.x = 0.01;
    shoulderPitch = atan2(-p_elbow_shoulder.z, p_elbow_shoulder.x);


    glm::mat3 r_after_shoulder_pitch = {cos(shoulderPitch), 0.0, -sin(shoulderPitch),
                                               0.0,         1.0,         0.0,
                                        sin(shoulderPitch), 0.0,  cos(shoulderPitch)};

    glm::vec3 p_elbow_shoulder_after_shoulder_pitch = glm::transpose(r_after_shoulder_pitch) * glm::vec3(p_elbow_shoulder);

    shoulderRoll = atan2(p_elbow_shoulder_after_shoulder_pitch.y, p_elbow_shoulder_after_shoulder_pitch.x);



    glm::mat3 r_shoulder_to_hand = glm::mat3(t_shoulder_to_hand);
    glm::mat3 r_elbow_to_shoulder = { cos(shoulderRoll)*cos(shoulderPitch),  sin(shoulderRoll), -cos(shoulderRoll)*sin(shoulderPitch),
                                     -sin(shoulderRoll)*cos(shoulderPitch),  cos(shoulderRoll),  sin(shoulderRoll)*sin(shoulderPitch),
                                               sin(shoulderPitch),                    0.0,               cos(shoulderPitch)};

    glm::vec3 e_forearm_elbow =  glm::transpose(r_elbow_to_shoulder) * r_shoulder_to_hand * e_l_forearm_hand;
    elbowYaw = -atan2(e_forearm_elbow.z, -e_forearm_elbow.y);


    glm::mat3 r_after_elbow_yaw = {1.0,       0.0,             0.0,
                                   0.0,  cos(elbowYaw),  sin(elbowYaw),
                                   0.0, -sin(elbowYaw),  cos(elbowYaw)};
                                   
    glm::vec3 e_forearm_elbow_after_elbow_yaw = glm::transpose(r_after_elbow_yaw) * e_forearm_elbow;

    elbowRoll = atan2(e_forearm_elbow_after_elbow_yaw.y, e_forearm_elbow_after_elbow_yaw.x);



    glm::mat3 r_after_elbow_roll = { cos(elbowRoll), sin(elbowRoll), 0.0,
                                    -sin(elbowRoll), cos(elbowRoll), 0.0,
                                       0.0,            0.0,          1.0};
 
    glm::mat3 r_wrist_to_elbow = glm::transpose(r_after_elbow_roll) * glm::transpose(r_after_elbow_yaw);

    glm::vec3 e_palm_wrist = r_wrist_to_elbow * r_elbow_to_shoulder * r_shoulder_to_hand * e_l_palm_hand;

    wristYaw = atan2(e_palm_wrist.y, -e_palm_wrist.z);


	m_JointAnglesMsg.joint_angles[AngleIndex::LElbowRoll] = elbowRoll;
	m_JointAnglesMsg.joint_angles[AngleIndex::LElbowYaw] = elbowYaw;
	m_JointAnglesMsg.joint_angles[AngleIndex::LShoulderPitch] = shoulderPitch;
	m_JointAnglesMsg.joint_angles[AngleIndex::LShoulderRoll] = shoulderRoll;
	m_JointAnglesMsg.joint_angles[AngleIndex::LWristYaw] = 0.0;//wristYaw;
}


void RosNode::SetRightArm(glm::mat4 transform){
    m_GroundToRightControllerTransform = transform;
    PublishTransformMsg(transform, "Ground", "RightController");

    double elbowRoll, elbowYaw, shoulderPitch, shoulderRoll, wristYaw;

    glm::mat4 t_shoulder_to_head = { 0.0,  0.0, -1.0, 0.000,
                                    -1.0,  0.0,  0.0, 0.210,
                                     0.0,  1.0,  0.0, 0.225,
                                     0.0,  0.0,  0.0, 1.000};
    t_shoulder_to_head = glm::transpose(t_shoulder_to_head) * glm::mat4(glm::mat3(m_GroundToHeadTransform));

    glm::mat4 t_shoulder_to_hand = t_shoulder_to_head * glm::inverse(m_GroundToHeadTransform) * transform;
    glm::vec4 p_elbow_shoulder = t_shoulder_to_hand * p_r_elbow_hand;
    if (p_elbow_shoulder.x < 0.01 ) p_elbow_shoulder.x = 0.01;
    shoulderPitch = atan2(-p_elbow_shoulder.z, p_elbow_shoulder.x);

    glm::mat3 r_after_shoulder_pitch = {cos(shoulderPitch), 0.0, -sin(shoulderPitch),
                                               0.0,         1.0,         0.0,
                                        sin(shoulderPitch), 0.0,  cos(shoulderPitch)};

    glm::vec3 p_elbow_shoulder_after_shoulder_pitch = glm::transpose(r_after_shoulder_pitch) * glm::vec3(p_elbow_shoulder);

    shoulderRoll = atan2(p_elbow_shoulder_after_shoulder_pitch.y, p_elbow_shoulder_after_shoulder_pitch.x);


    glm::mat3 r_shoulder_to_hand = glm::mat3(t_shoulder_to_hand);
    glm::mat3 r_elbow_to_shoulder = { cos(shoulderRoll)*cos(shoulderPitch),  sin(shoulderRoll), -cos(shoulderRoll)*sin(shoulderPitch),
                                     -sin(shoulderRoll)*cos(shoulderPitch),  cos(shoulderRoll),  sin(shoulderRoll)*sin(shoulderPitch),
                                               sin(shoulderPitch),                    0.0,               cos(shoulderPitch)};

    glm::vec3 e_forearm_elbow =  glm::transpose(r_elbow_to_shoulder) * r_shoulder_to_hand * e_r_forearm_hand;
    elbowYaw = atan2(e_forearm_elbow.z, e_forearm_elbow.y);

    glm::mat3 r_after_elbow_yaw = {1.0,       0.0,             0.0,
                                   0.0,  cos(elbowYaw),  sin(elbowYaw),
                                   0.0, -sin(elbowYaw),  cos(elbowYaw)};                    
    glm::vec3 e_forearm_elbow_after_elbow_yaw = glm::transpose(r_after_elbow_yaw) * e_forearm_elbow;
    elbowRoll = atan2(e_forearm_elbow_after_elbow_yaw.y, e_forearm_elbow_after_elbow_yaw.x);



    glm::mat3 r_after_elbow_roll = { cos(elbowRoll), sin(elbowRoll), 0.0,
                                    -sin(elbowRoll), cos(elbowRoll), 0.0,
                                       0.0,            0.0,          1.0};
 
    glm::mat3 r_wrist_to_elbow = glm::transpose(r_after_elbow_roll) * glm::transpose(r_after_elbow_yaw);

    glm::vec3 e_palm_wrist = r_wrist_to_elbow * r_elbow_to_shoulder * r_shoulder_to_hand * e_r_palm_hand;

    wristYaw = atan2(e_palm_wrist.y, -e_palm_wrist.z);


	m_JointAnglesMsg.joint_angles[AngleIndex::RElbowRoll] = elbowRoll;
	m_JointAnglesMsg.joint_angles[AngleIndex::RElbowYaw] = elbowYaw;
	m_JointAnglesMsg.joint_angles[AngleIndex::RShoulderPitch] = shoulderPitch;
	m_JointAnglesMsg.joint_angles[AngleIndex::RShoulderRoll] = shoulderRoll;
	m_JointAnglesMsg.joint_angles[AngleIndex::RWristYaw] = 0.0;//wristYaw;
}

void RosNode::SetRightHand(float gripperVal){
    m_JointAnglesMsg.joint_angles[AngleIndex::RHand] = 1.0f - gripperVal;
}

void RosNode::SetLeftHand(float gripperVal){
    m_JointAnglesMsg.joint_angles[AngleIndex::LHand] = 1.0f - gripperVal;
}

//new code for robot dpad movement 5/2022

void RosNode::SetDpadUp(bool DpadForward) { // these 4 functions call to pepper_motion_controll.py. There a subscriber takes this informaton.

    up = DpadForward;
}

void RosNode::SetDpadDown(bool DpadBackward) {

    down = DpadBackward;

}

void RosNode::SetDpadRight(bool DpadRight) {

    right = DpadRight;
    
}

void RosNode::SetDpadLeft(bool DpadLeft) {

    left = DpadLeft;

}


void RosNode::CalibrateRightArm(glm::mat4 transform){
    glm::mat4 t_shoulder_to_head = { 0.0,  0.0, -1.0, 0.100,
                                    -1.0,  0.0,  0.0, 0.210,
                                     0.0,  1.0,  0.0, 0.225,
                                     0.0,  0.0,  0.0, 1.000};
    t_shoulder_to_head = glm::transpose(t_shoulder_to_head) * glm::mat4(glm::mat3(m_GroundToHeadTransform));

    glm::mat4 t_shoulder_to_hand = t_shoulder_to_head * glm::inverse(m_GroundToHeadTransform) * transform;
    glm::mat4 t_hand_to_shoulder = glm::inverse(t_shoulder_to_hand);
    glm::vec3 e_shoulder_hand = glm::vec3(t_hand_to_shoulder[3]);

    p_r_elbow_hand = glm::vec4(e_shoulder_hand * 0.5f, 1.0f);
    e_r_forearm_hand = -glm::normalize(glm::vec3(p_r_elbow_hand));
    e_r_palm_hand = t_hand_to_shoulder * glm::vec4(0.0f, 0.0f, -1.0f, 1.0f);
}


void RosNode::CalibrateLeftArm(glm::mat4 transform){
    glm::mat4 t_shoulder_to_head = { 0.0,  0.0, -1.0,  0.000,
                                    -1.0,  0.0,  0.0, -0.210,
                                     0.0,  1.0,  0.0,  0.225,
                                     0.0,  0.0,  0.0,  1.000};
    t_shoulder_to_head = glm::transpose(t_shoulder_to_head) * glm::mat4(glm::mat3(m_GroundToHeadTransform));

    glm::mat4 t_shoulder_to_hand = t_shoulder_to_head * glm::inverse(m_GroundToHeadTransform) * transform;
    glm::mat4 t_hand_to_shoulder = glm::inverse(t_shoulder_to_hand);
    glm::vec3 e_shoulder_hand = glm::vec3(t_hand_to_shoulder[3]);

    p_l_elbow_hand = glm::vec4(e_shoulder_hand * 0.5f, 1.0f);
    e_l_forearm_hand = -glm::normalize(glm::vec3(p_l_elbow_hand));
    e_l_palm_hand = t_hand_to_shoulder * glm::vec4(0.0f, 0.0f, -1.0f, 1.0f);
}

void RosNode::SetHeadNew(glm::mat4 transform){

}

void RosNode::CalibrateRightArmNew(glm::mat4 transform){

}

void RosNode::CalibrateLeftArmNew(glm::mat4 transform){

}

void RosNode::SetLeftArmNew(glm::mat4 transform){

}

void RosNode::SetRightArmNew(glm::mat4 transform){
 
}

void RosNode::OnUpdate(){
    ros::spinOnce();
}

glm::quat RosNode::RotationBetweenVectors(glm::vec3 start, glm::vec3 dest){
	start = glm::normalize(start);
	dest = glm::normalize(dest);
	
	float cosTheta = glm::dot(start, dest);
	glm::vec3 rotationAxis;
	
	if (cosTheta < -1 + 0.001f){
		// special case when vectors in opposite directions :
		// there is no "ideal" rotation axis
		// So guess one; any will do as long as it's perpendicular to start
		// This implementation favors a rotation around the Up axis,
		// since it's often what you want to do.
		rotationAxis = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), start);
		if (glm::length2(rotationAxis) < 0.01 ) // bad luck, they were parallel, try again!
			rotationAxis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), start);
		
		rotationAxis = normalize(rotationAxis);
		return angleAxis(glm::radians(180.0f), rotationAxis);
	}

	// Implementation from Stan Melax's Game Programming Gems 1 article
	rotationAxis = glm::cross(start, dest);

	float s = sqrt( (1+cosTheta)*2 );
	float invs = 1 / s;

	return glm::quat(
		s * 0.5f, 
		rotationAxis.x * invs,
		rotationAxis.y * invs,
		rotationAxis.z * invs
	);
}

// Returns a quaternion that will make your object looking towards 'direction'.
// Similar to RotationBetweenVectors, but also controls the vertical orientation.
// This assumes that at rest, the object faces +X.
// Beware, the first parameter is a direction, not the target point !
glm::quat RosNode::LookAt(glm::vec3 direction, glm::vec3 desiredUp){
	if (glm::length2(direction) < 0.0001f )
		return glm::quat();

	// Recompute desiredUp so that it's perpendicular to the direction
	// You can skip that part if you really want to force desiredUp
	glm::vec3 right = glm::cross(direction, desiredUp);
	desiredUp = glm::cross(right, direction);
	
	// Find the rotation between the front of the object (that we assume towards +X,
	// but this depends on your model) and the desired direction
	glm::quat rot1 = RotationBetweenVectors(glm::vec3(1.0f, 0.0f, 0.0f), direction);
	// Because of the 1rst rotation, the up is probably completely screwed up. 
	// Find the rotation between the "up" of the rotated object, and the desired up
	glm::vec3 newUp = rot1 * glm::vec3(0.0f, 0.0f, 1.0f);
	glm::quat rot2 = RotationBetweenVectors(newUp, desiredUp);
	
	// Apply them
	return rot2 * rot1; // remember, in reverse order.
}

void RosNode::PublishTransform(glm::mat4 transform, std::string parent, std::string child){
    tf::Transform t;

    t.setOrigin(tf::Vector3(transform[3][0], transform[3][1], transform[3][2]));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(transform[0][0], transform[1][0], transform[2][0],
                  transform[0][1], transform[1][1], transform[2][1],
                  transform[0][2], transform[1][2], transform[2][2]);
    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);
    t.setRotation(tfqt);

    m_TransformBroadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), parent, child));
}

void RosNode::PublishTransformMsg(glm::mat4 transform, std::string parent, std::string child){
    geometry_msgs::TransformStamped msg;

    msg.header.frame_id = parent;
    msg.header.stamp = ros::Time::now();
    msg.child_frame_id = child;

    geometry_msgs::Vector3 translation;
    translation.x = transform[3][0];
    translation.y = transform[3][1];
    translation.z = transform[3][2];
    msg.transform.translation = translation;

    tf::Matrix3x3 tf3d;
    tf3d.setValue(transform[0][0], transform[1][0], transform[2][0],
                  transform[0][1], transform[1][1], transform[2][1],
                  transform[0][2], transform[1][2], transform[2][2]);
    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);
    geometry_msgs::Quaternion rotation;
    rotation.w = tfqt.getW();
    rotation.x = tfqt.getX();
    rotation.y = tfqt.getY();
    rotation.z = tfqt.getZ();
    msg.transform.rotation = rotation;

    m_TransformMsgPub->publish(msg);
}