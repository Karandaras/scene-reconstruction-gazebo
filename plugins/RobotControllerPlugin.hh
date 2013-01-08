#ifndef __GAZEBO_ROBOTCONTROLLER_PLUGIN_HH__
#define __GAZEBO_ROBOTCONTROLLER_PLUGIN_HH__

#include <vector>

#include "common/common.h"
#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "gazebo.hh"

namespace gazebo
{
  /** @class RobotControllerPlugin "robotcontrollerplugin.h"
   *  RobotController Plugin for Scene Reconstruction using the Gazebo simulator
   *  @author Bastian Klingen
   */
  class RobotControllerPlugin : public ModelPlugin
  {
    public: 
      /** Constructor */
      RobotControllerPlugin();

    private:
      /** struct for jointdata */
      struct JointData {
          /** jointname inside the simulator */
          std::string simulator_name;
          /** name of a second affected joint inside the simulator */
          std::string simulator_name2;
          /** jointname inside the robot framework */
          std::string robot_name;
          /** is this joint a gripper */
          bool gripper;
          /** angle offset between simulator and robot */
          double offset;
          /** angle offset between simulator and robot for the second joint*/
          double offset2;
          /** joint angle in simulator */
          double simulator_angle;
          /** joint angle in simulator for the second joint */
          double simulator_angle2;
          /** joint angle in robot framework */
          double robot_angle;
          /** factor to adjust gear transmission ratios between simulator and robot */
          double factor;
      };

      /** struct for buffered joint commands */
      struct JointCommand {
          /** time when this command triggers */
          common::Time                 controltime;
          /** new positions for the joints */
          std::map<std::string,double> positions;

          /** comparison operator for find */
          bool operator<(JointCommand comp) const {
            return controltime<comp.controltime;
          }
      };

      /** struct for buffered robot commands */
      struct RobotCommand {
          /** time when this command triggers */
          common::Time                 controltime;
          /** new positions for the robot */
          math::Pose                   pose;

          /** comparison operator for find */
          bool operator<(RobotCommand comp) const {
            return controltime<comp.controltime;
          }
      };

      transport::NodePtr                          node;
      transport::SubscriberPtr                    controlSub, 
                                                  srguiSub,
                                                  initSub,
                                                  drawingSub,
                                                  positionSub,
                                                  anglesSub;
      transport::PublisherPtr                     srguiPub,
                                                  offsetPub,
                                                  drawingPub,
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
      bool                                        __available,
                                                  __reset,
                                                  bufferpreview_joint,
                                                  bufferpreview_pose;

    public: 
      /** initialisation of the plugin */
      virtual void Init();
      /** reset of the plugin */
      virtual void Reset();
      /** loading routine of the plugin */
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
      /** Processes the message queue and builds the buffer
        */
      void ProcessControlMsgs();
      /** Updates the current joint angles using the buffer
        * @param now The current time for reference with the buffer timestamps
        */
      void ControlJoints(common::Time now);
      /** Updates the current robot position using the buffer
        * @param now The current time for reference with the buffer timestamps
        */
      void ControlRobot(common::Time now);
      /** Update procedure connected to Gazebo's update
        */
      void OnUpdate();
      /** Callback for received Message_V messages to enqueue them for buffering
        * @param _msg Message_V message containing data for buffering
        */
      void OnControlMsg(ConstMessage_VPtr &_msg);
      /** Callback for received SceneRobotController message to set initial position and angles
        * @param _msg SceneRobotController message containing data
        */
      void OnInitMsg(ConstSceneRobotControllerPtr &_msg);
      /** Procedure to set the robot to initial values, called on startup and after reset
        */
      void InitMsg();
      /** Callback for received Request messages
        * @param _msg Request message
        */
      void OnRequestMsg(ConstRequestPtr &_msg);
      /** Callback for received Drawing messages to position them relative to the robot
        * @param _msg Drawing message
        */
      void OnDrawingMsg(ConstDrawingPtr &_msg);
      /** Callback for received SceneRobot messages to visualize buffer content
        * @param _msg SceneRobot message containing needed data for visualization
        */
      void OnPositionMsg(ConstSceneRobotPtr &_msg);
      /** Callback for received SceneJoint messages to visualize buffer content
        * @param _msg SceneJoint message containing needed data for visualization
        */
      void OnAnglesMsg(ConstSceneJointPtr &_msg);
  };
}
#endif
