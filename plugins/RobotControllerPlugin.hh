#ifndef __GAZEBO_ROBOTCONTROLLER_PLUGIN_HH__
#define __GAZEBO_ROBOTCONTROLLER_PLUGIN_HH__

#include "common/common.h"
#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "gazebo.hh"

namespace gazebo
{
  class RobotControllerPlugin : public ModelPlugin
  {
    typedef struct {
        std::string simulator_name;
        std::string robot_name;
        double offset;
        double simulator_angle;
        double robot_angle;
    } JointData;

    public: RobotControllerPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnSceneJointMsg(ConstSceneJointPtr &_msg);
    private: void OnSceneChangeMsg(ConstSceneRobotControllerPtr &_msg);
    private: void OnRequestMsg(ConstRequestPtr &_msg);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr jointSub, srguiSub, changeJointSub;
    private: transport::PublisherPtr srguiPub;

    private: physics::ModelPtr model;
    private: std::map<std::string, JointData> jointdata;
    private: std::map<std::string, JointData>::iterator jointiter;
  };
}
#endif
