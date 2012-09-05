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
    public: 
      RobotControllerPlugin();

    private:
      typedef struct {
          std::string simulator_name;
          std::string robot_name;
          double offset;
          double simulator_angle;
          double robot_angle;
      } JointData;

      typedef struct {
          common::Time                 controltime;
          std::map<std::string,double> positions;
      } ControlCommand;

      transport::NodePtr                          node;
      transport::SubscriberPtr                    jointSub, 
                                                  srguiSub,
                                                  changeJointSub,
                                                  statusSub;
      transport::PublisherPtr                     srguiPub,
                                                  statusPub;

      common::Time                                next_control;

      physics::ModelPtr                           model;
      boost::mutex                               *receiveMutex;
      std::map<std::string, JointData>            jointdata;
      std::map<std::string, JointData>::iterator  jointiter;
      std::list<msgs::Message_V>                  controlMsgs;
      std::list<ControlCommand>       controlList;

    public: 
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Init();

    private:
      void ProcessJointMsgs();
      void ControlRobot(common::Time now);
      void OnUpdate();
      void OnSceneJointMsg(ConstMessage_VPtr &_msg);
      void OnSceneChangeMsg(ConstSceneRobotControllerPtr &_msg);
      void OnRequestMsg(ConstRequestPtr &_msg);
      void OnStatusMsg(ConstRequestPtr &_msg);
  };
}
#endif
