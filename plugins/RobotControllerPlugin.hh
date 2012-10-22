#ifndef __GAZEBO_ROBOTCONTROLLER_PLUGIN_HH__
#define __GAZEBO_ROBOTCONTROLLER_PLUGIN_HH__

#include <vector>

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
      struct JointData {
          std::string simulator_name;
          std::string simulator_name2;
          std::string robot_name;
          bool gripper;
          double offset;
          double simulator_angle;
          double robot_angle;
          double factor;
      };

      struct JointCommand {
          common::Time                 controltime;
          std::map<std::string,double> positions;

          bool operator<(JointCommand comp) const {
            return controltime<comp.controltime;
          }
      };

      struct RobotCommand {
          common::Time                 controltime;
          math::Pose                   pose;

          bool operator<(RobotCommand comp) const {
            return controltime<comp.controltime;
          }
      };

      transport::NodePtr                          node;
      transport::SubscriberPtr                    controlSub, 
                                                  srguiSub,
                                                  initSub,
                                                  statusSub;
      transport::PublisherPtr                     srguiPub,
                                                  offsetPub,
                                                  bufferPub,
                                                  statusPub;

      common::Time                                next_joint_control,
                                                  next_robot_control,
                                                  lastinfo;

      physics::ModelPtr                           model;
      boost::mutex                               *receiveMutex,
                                                 *robotMutex,
                                                 *jointMutex;
      std::map<std::string, JointData>            jointdata;
      std::map<std::string, JointData>::iterator  jointiter;
      std::list<msgs::Message_V>                  controlMsgs;
      std::list<JointCommand>                     jointControlList;
      std::list<RobotCommand>                     robotControlList;

      math::Vector3                               position_offset;

      std::vector<event::ConnectionPtr>           connections;

      physics::WorldPtr                           world;
      boost::shared_ptr<gazebo::msgs::SceneRobotController const> initMsg;

      std::map<std::string,double>                currentjointpositions;
      math::Pose                                  currentpose;
      bool                                        setpose;

    public: 
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Init();
      virtual void Reset();

    private:
      void ProcessControlMsgs();
      void ControlJoints(common::Time now);
      void ControlRobot(common::Time now);
      void OnUpdate();
      void OnControlMsg(ConstMessage_VPtr &_msg);
      void OnInitMsg(ConstSceneRobotControllerPtr &_msg);
      void InitMsg();
      void OnRequestMsg(ConstRequestPtr &_msg);
      void OnStatusMsg(ConstRequestPtr &_msg);
      void fill_joint_buffer_msg(msgs::Message_V &_msg);
      void fill_position_buffer_msg(msgs::Message_V &_msg);
  };
}
#endif
