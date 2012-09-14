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
          std::string robot_name;
          double offset;
          double simulator_angle;
          double robot_angle;
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
                                                  setupSub,
                                                  initSub,
                                                  statusSub;
      transport::PublisherPtr                     srguiPub,
                                                  offsetPub,
                                                  statusPub;

      common::Time                                next_joint_control,
                                                  next_robot_control;

      physics::ModelPtr                           model;
      boost::mutex                               *receiveMutex,
                                                 *robotMutex,
                                                 *jointMutex;
      std::map<std::string, JointData>            jointdata;
      std::map<std::string, JointData>::iterator  jointiter;
      std::list<msgs::Message_V>                  controlMsgs;
      std::list<JointCommand>                     jointControlList;
      std::list<RobotCommand>                     robotControlList;

      double                                      position_x_offset,
                                                  position_y_offset,
                                                  position_z_offset,
                                                  orientation_w_offset,
                                                  orientation_x_offset,
                                                  orientation_y_offset,
                                                  orientation_z_offset;

      std::vector<event::ConnectionPtr>           connections;

      physics::WorldPtr                           world;

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
      void OnSetupMsg(ConstSceneRobotControllerPtr &_msg);
      void OnInitMsg(ConstSceneRobotControllerPtr &_msg);
      void OnRequestMsg(ConstRequestPtr &_msg);
      void OnStatusMsg(ConstRequestPtr &_msg);
  };
}
#endif
