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

    public: 
      RobotControllerPlugin();

    private:
      transport::NodePtr                          node;
      transport::SubscriberPtr                    jointSub, 
                                                  srguiSub,
                                                  changeJointSub,
                                                  statusSub;
      transport::PublisherPtr                     srguiPub,
                                                  statusPub;

      physics::ModelPtr                           model;
      std::map<std::string, JointData>            jointdata;
      std::map<std::string, JointData>::iterator  jointiter;

    public: 
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Init();

    private:
      void OnSceneJointMsg(ConstSceneJointPtr &_msg);
      void OnSceneChangeMsg(ConstSceneRobotControllerPtr &_msg);
      void OnRequestMsg(ConstRequestPtr &_msg);
      void OnStatusMsg(ConstRequestPtr &_msg);
  };
}
#endif
