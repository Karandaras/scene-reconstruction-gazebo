#include <float.h>

#include "physics/physics.h"
#include "transport/transport.h"
#include "plugins/RobotControllerPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(RobotControllerPlugin)

/////////////////////////////////////////////////
RobotControllerPlugin::RobotControllerPlugin()
{
}

/////////////////////////////////////////////////
void RobotControllerPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = this->model->GetWorld();
  this->receiveMutex = new boost::mutex();
  this->robotMutex = new boost::mutex();
  this->jointMutex = new boost::mutex();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  this->controlSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/"), &RobotControllerPlugin::OnControlMsg, this);
  this->setupSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Setup"), &RobotControllerPlugin::OnSetupMsg, this);
  this->srguiSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Request"), &RobotControllerPlugin::OnRequestMsg, this);
  this->statusSub = this->node->Subscribe(std::string("~/SceneReconstruction/GUI/Availability/Request/RobotController"), &RobotControllerPlugin::OnStatusMsg, this);
  this->srguiPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/RobotController/Response"));
  this->statusPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));

  sdf::ElementPtr jointElem;

  if (!_sdf->HasElement("rcjoint"))
    gzerr << "missing <rcjoint> element(s)\n";
  else {
    jointElem = _sdf->GetElement("rcjoint");
    std::string rcjoint;
    while(jointElem) {
      std::string gname;
      std::string fname;
      std::string offtmp;
      double off;

      jointElem->GetValue()->Get(rcjoint);

      if(_sdf->HasElement("gname_"+rcjoint)) {
        if(!_sdf->GetElement("gname_"+rcjoint)->GetValue()->Get(gname)) {
          gzerr << "<gname_" << rcjoint << "> is not a string, leaving out this <rcjoint>\n";
        } else {
          if(_sdf->HasElement("fname_"+rcjoint)) {
            if(!_sdf->GetElement("fname_"+rcjoint)->GetValue()->Get(fname)) {
              gzerr << "<fname_" << rcjoint << "> not a string, defaulting to gname:" << gname << "\n";
            fname = gname;
            }
          } else {
            fname = gname;
          }
          if(_sdf->HasElement("offset_"+rcjoint)) {
            if(!_sdf->GetElement("offset_"+rcjoint)->GetValue()->Get(offtmp)) {
              gzerr << "<offset_" << rcjoint << "> not readable, defaulting to 0.0\n";
              off = 0.0;
            }
            else {
              char* t;
              off = strtod(offtmp.c_str(), &t);
              if(*t != 0) {
                gzerr << "<offset_" << rcjoint << "> not a double, defaulting to 0.0\n";
                off = 0.0;
              }  
            }
          } else {
            off = 0.0;
          }

          physics::JointPtr j = _model->GetJoint(gname);
          if(!j) {
            gzerr << "unable to find joint " << gname << "\n";
          } else {
            double simangle = j->GetAngle(0).GetAsRadian();

            this->jointdata[fname].simulator_name = gname;
            this->jointdata[fname].robot_name = fname;
            this->jointdata[fname].offset = off;
            this->jointdata[fname].simulator_angle = simangle;
            this->jointdata[fname].robot_angle = simangle-off;
          }
        }
      } else {
        gzerr << "missing required element <gname_" << rcjoint << ">, leaving out this <rcjoint>\n";
      }
      jointElem = jointElem->GetNextElement("rcjoint");
    }
  }

  // connect update to worldupdate
  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
          boost::bind(&RobotControllerPlugin::OnUpdate, this)));

  msgs::Response response;
  response.set_id(-1);
  response.set_request("status");
  response.set_response("RobotController");
  this->statusPub->Publish(response);
}

/////////////////////////////////////////////////
void RobotControllerPlugin::Init()
{
  this->next_joint_control = common::Time(world->GetSimTime());
  this->next_robot_control = common::Time(world->GetSimTime());
}

void RobotControllerPlugin::Reset()
{
  this->next_joint_control = common::Time(world->GetSimTime());
  this->next_robot_control = common::Time(world->GetSimTime());
  jointControlList.clear();
  robotControlList.clear();
  controlMsgs.clear();
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnUpdate()
{
  if(this->world->IsPaused())
    return;

  common::Time now = common::Time(world->GetSimTime());
  this->ProcessControlMsgs();
  this->ControlJoints(now);
  this->ControlRobot(now);
}

void RobotControllerPlugin::ControlJoints(common::Time now) {
  boost::mutex::scoped_lock lock(*this->jointMutex);
  if(now >= this->next_joint_control) {
    this->next_joint_control = now + common::Time(1);
    std::list< JointCommand >::iterator it;
    std::list< JointCommand > newJointList;
    for(it = jointControlList.begin(); it != jointControlList.end(); it++) {
      if(it->controltime < now) {
        this->model->SetJointPositions(it->positions);
      }
      else {
        if(it->controltime < this->next_joint_control) {
          this->next_joint_control = it->controltime;
        }
        newJointList.push_back(*it);
      }
    }
    jointControlList = newJointList;
    jointControlList.sort();
  }
}

void RobotControllerPlugin::ControlRobot(common::Time now) {
  boost::mutex::scoped_lock lock(*this->robotMutex);
  if(now >= this->next_robot_control) {
    this->next_robot_control = now + common::Time(1);
    std::list< RobotCommand >::iterator it;
    std::list< RobotCommand > newRobotList;
    for(it = robotControlList.begin(); it != robotControlList.end(); it++) {
      if(it->controltime < now) {
        this->model->SetWorldPose(it->pose);
      }
      else {
        if(it->controltime < this->next_robot_control) {
          this->next_robot_control = it->controltime;
        }
        newRobotList.push_back(*it);
      }
    }
    robotControlList = newRobotList;
    robotControlList.sort();
  }
}

void RobotControllerPlugin::ProcessControlMsgs() {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  boost::mutex::scoped_lock lockr(*this->robotMutex);
  boost::mutex::scoped_lock lockj(*this->jointMutex);
  std::list<msgs::Message_V>::iterator _msg;
  for (_msg = this->controlMsgs.begin(); _msg != this->controlMsgs.end(); ++_msg) {
    msgs::SceneJoint joint;
    msgs::SceneRobot robot;
    if(_msg->msgtype() == joint.GetTypeName()) {
      int n_msgs = _msg->msgsdata_size();
      for(int n=0; n<n_msgs; n++) {
        joint.ParseFromString(_msg->msgsdata(n));
        std::map<std::string,double> positions;

        int joints = joint.joint_size();
        if(joints <= (int)this->model->GetJointCount()) {
          if(joints == joint.angle_size()) {
            for(int i=0; i<joints; i++) {
              this->jointiter = this->jointdata.find(joint.joint(i));
              if(this->jointiter != jointdata.end()) {
                positions[this->jointiter->second.simulator_name] = this->jointiter->second.offset+joint.angle(i);
       	        this->jointiter->second.simulator_angle = this->jointiter->second.offset+joint.angle(i);
                this->jointiter->second.robot_angle = joint.angle(i);
              }
            }
            this->jointiter = this->jointdata.end();

            JointCommand c;
            if(joint.has_controltime())
              c.controltime = common::Time(joint.controltime());
            else
              c.controltime = common::Time(world->GetSimTime());
            c.positions = positions;

            this->jointControlList.push_back(c);
          } else {
            gzerr << "number of joints differs from number of angles\n";
          }
        } else {
          gzerr << "message tries to alter more joints then available\n";
        }
      }
    }
    else if(_msg->msgtype() == robot.GetTypeName()) {
      int n_msgs = _msg->msgsdata_size();
      for(int n=0; n<n_msgs; n++) {
        robot.ParseFromString(_msg->msgsdata(n));
        RobotCommand c;
        if(robot.has_controltime())
          c.controltime = common::Time(robot.controltime());
        else
          c.controltime = common::Time(world->GetSimTime());
        c.pose.pos.x = robot.pos_x();
        c.pose.pos.y = robot.pos_y();
        if(robot.has_pos_z())
          c.pose.pos.z = robot.pos_z();
        else {
          c.pose.pos.z = 0;
        }

        if(robot.has_rot_w())
          c.pose.rot.w = robot.rot_w();
        if(robot.has_rot_x())
          c.pose.rot.x = robot.rot_x();
        if(robot.has_rot_y())
          c.pose.rot.y = robot.rot_y();
        if(robot.has_rot_z())
          c.pose.rot.z = robot.rot_z();
        

        this->robotControlList.push_back(c);
      }
    }
    else {
      gzerr << "message of unknown type\n";
    }
  }

  jointControlList.sort();
  robotControlList.sort();
  controlMsgs.clear();
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnSetupMsg(ConstSceneRobotControllerPtr &_msg) {
  std::map<std::string,double> positions;
  int sn, rn, o, sa, ra;
  sn = _msg->simulator_name_size();
  rn = _msg->robot_name_size();
  o  = _msg->offset_size();
  sa = _msg->simulator_angle_size();
  ra = _msg->robot_angle_size();

  if(sn == rn && rn == o && o == sa && sa == ra && ra == sn) {
    for(int i=0; i<sn; i++) {
      jointiter = jointdata.find(_msg->robot_name(i));
      if(jointiter != jointdata.end()) {
        jointiter->second.simulator_name = _msg->simulator_name(i);
        jointiter->second.offset = _msg->offset(i);
        jointiter->second.simulator_angle = _msg->simulator_angle(i);
        jointiter->second.robot_angle = _msg->robot_angle(i);
        positions[jointiter->second.simulator_name] = jointiter->second.simulator_angle;
      }
    }

    jointiter = jointdata.end();
    this->model->SetJointPositions(positions);
  }
  else {
    gzerr << "not all fields set for all joints\n";
  }

  math::Pose pose;

  pose.pos.x = _msg->pos_x();
  pose.pos.y = _msg->pos_y();
  if(_msg->has_pos_z())
    pose.pos.z = _msg->pos_z();
  else {
    pose.pos.z = 0;
  }

  if(_msg->has_rot_w())
    pose.rot.w = _msg->rot_w();
  if(_msg->has_rot_x())
    pose.rot.x = _msg->rot_x();
  if(_msg->has_rot_y())
    pose.rot.y = _msg->rot_y();
  if(_msg->has_rot_z())
    pose.rot.z = _msg->rot_z();
  
  this->model->SetWorldPose(pose);
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnControlMsg(ConstMessage_VPtr &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  controlMsgs.push_back(*_msg);
}

void RobotControllerPlugin::OnRequestMsg(ConstRequestPtr &_msg) {
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  if(_msg->request() == "controller_info") {
    msgs::SceneRobotController src;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response("success");
    response.set_type(src.GetTypeName());
    
    for(jointiter = jointdata.begin(); jointiter != jointdata.end(); jointiter++) {
        src.add_simulator_name(jointiter->second.simulator_name);
        src.add_robot_name(jointiter->second.robot_name);
        src.add_offset(jointiter->second.offset);
        src.add_simulator_angle(jointiter->second.simulator_angle);
	      src.add_robot_angle(jointiter->second.robot_angle);
    }
    math::Pose pose = this->model->GetWorldPose();
    src.set_pos_x(pose.pos.x);
    src.set_pos_y(pose.pos.y);
    src.set_pos_z(pose.pos.z);
    src.set_rot_w(pose.rot.w);
    src.set_rot_x(pose.rot.y);
    src.set_rot_y(pose.rot.y);
    src.set_rot_z(pose.rot.z);

    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    this->srguiPub->Publish(response);
  }
  else {
    response.set_response("unknown");
    msgs::String src;
    response.set_type(src.GetTypeName());
    src.set_data("the given request is unknown to the robotcontroller");
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);
    this->srguiPub->Publish(response);
  }
}

void RobotControllerPlugin::OnStatusMsg(ConstRequestPtr &_msg) {
  if(_msg->request() == "status") {
    msgs::Response response;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response("RobotController");
    this->statusPub->Publish(response);
  }
}
