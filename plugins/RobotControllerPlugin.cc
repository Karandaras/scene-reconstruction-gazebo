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

  this->position_x_offset = 0.0;
  this->position_y_offset = 0.0;
  this->position_z_offset = 0.0;
  this->orientation_w_offset = 0.0;
  this->orientation_x_offset = 0.0;
  this->orientation_y_offset = 0.0;
  this->orientation_z_offset = 0.0;

  if(_sdf->HasElement("settings_position_x_offset")) {
    std::string tmp_x_offset;
    double x_offset;
    if(!_sdf->GetElement("settings_position_x_offset")->GetValue()->Get(tmp_x_offset)) {
  	  gzwarn << "<settings_position_x_offset> is not readable, defaulting to 0.0\n";
    }
    else {
      char* t;
      x_offset = strtod(tmp_x_offset.c_str(), &t);
      if(*t != 0) {
        gzwarn << "<settings_position_x_offset> not a double, defaulting to 0.0\n";
      }
      else {
        this->position_x_offset = x_offset;
      }
    }
  }

  if(_sdf->HasElement("settings_position_y_offset")) {
    std::string tmp_y_offset;
    double y_offset;
    if(!_sdf->GetElement("settings_position_y_offset")->GetValue()->Get(tmp_y_offset)) {
  	  gzwarn << "<settings_position_y_offset> is not readable, defaulting to 0.0\n";
    }
    else {
      char* t;
      y_offset = strtod(tmp_y_offset.c_str(), &t);
      if(*t != 0) {
        gzwarn << "<settings_position_y_offset> not a double, defaulting to 0.0\n";
      }
      else {
        this->position_y_offset = y_offset;
      }
    }
  }

  if(_sdf->HasElement("settings_position_z_offset")) {
    std::string tmp_z_offset;
    double z_offset;
    if(!_sdf->GetElement("settings_position_z_offset")->GetValue()->Get(tmp_z_offset)) {
  	  gzwarn << "<settings_position_z_offset> is not readable, defaulting to 0.0\n";
    }
    else {
      char* t;
      z_offset = strtod(tmp_z_offset.c_str(), &t);
      if(*t != 0) {
        gzwarn << "<settings_position_z_offset> not a double, defaulting to 0.0\n";
      }
      else {
        this->position_z_offset = z_offset;
      }
    }
  }

  if(_sdf->HasElement("settings_orientation_w_offset")) {
    std::string tmp_w_offset;
    double w_offset;
    if(!_sdf->GetElement("settings_orientation_w_offset")->GetValue()->Get(tmp_w_offset)) {
  	  gzwarn << "<settings_orientation_w_offset> is not readable, defaulting to 0.0\n";
    }
    else {
      char* t;
      w_offset = strtod(tmp_w_offset.c_str(), &t);
      if(*t != 0) {
        gzwarn << "<settings_orientation_w_offset> not a double, defaulting to 0.0\n";
      }
      else {
        this->orientation_w_offset = w_offset;
      }
    }
  }

  if(_sdf->HasElement("settings_orientation_x_offset")) {
    std::string tmp_x_offset;
    double x_offset;
    if(!_sdf->GetElement("settings_orientation_x_offset")->GetValue()->Get(tmp_x_offset)) {
  	  gzwarn << "<settings_orientation_x_offset> is not readable, defaulting to 0.0\n";
    }
    else {
      char* t;
      x_offset = strtod(tmp_x_offset.c_str(), &t);
      if(*t != 0) {
        gzwarn << "<settings_orientation_x_offset> not a double, defaulting to 0.0\n";
      }
      else {
        this->orientation_x_offset = x_offset;
      }
    }
  }

  if(_sdf->HasElement("settings_orientation_y_offset")) {
    std::string tmp_y_offset;
    double y_offset;
    if(!_sdf->GetElement("settings_orientation_y_offset")->GetValue()->Get(tmp_y_offset)) {
  	  gzwarn << "<settings_orientation_y_offset> is not readable, defaulting to 0.0\n";
    }
    else {
      char* t;
      y_offset = strtod(tmp_y_offset.c_str(), &t);
      if(*t != 0) {
        gzwarn << "<settings_orientation_y_offset> not a double, defaulting to 0.0\n";
      }
      else {
        this->orientation_y_offset = y_offset;
      }
    }
  }

  if(_sdf->HasElement("settings_orientation_z_offset")) {
    std::string tmp_z_offset;
    double z_offset;
    if(!_sdf->GetElement("settings_orientation_z_offset")->GetValue()->Get(tmp_z_offset)) {
  	  gzwarn << "<settings_orientation_z_offset> is not readable, defaulting to 0.0\n";
    }
    else {
      char* t;
      z_offset = strtod(tmp_z_offset.c_str(), &t);
      if(*t != 0) {
        gzwarn << "<settings_orientation_z_offset> not a double, defaulting to 0.0\n";
      }
      else {
        this->orientation_z_offset = z_offset;
      }
    }
  }

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
              gzwarn << "<fname_" << rcjoint << "> not a string, defaulting to gname:" << gname << "\n";
            fname = gname;
            }
          } else {
            fname = gname;
          }
          if(_sdf->HasElement("offset_"+rcjoint)) {
            if(!_sdf->GetElement("offset_"+rcjoint)->GetValue()->Get(offtmp)) {
              gzwarn << "<offset_" << rcjoint << "> not readable, defaulting to 0.0\n";
              off = 0.0;
            }
            else {
              char* t;
              off = strtod(offtmp.c_str(), &t);
              if(*t != 0) {
                gzwarn << "<offset_" << rcjoint << "> not a double, defaulting to 0.0\n";
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

  this->controlSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/"), &RobotControllerPlugin::OnControlMsg, this);
  this->setupSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Setup"), &RobotControllerPlugin::OnSetupMsg, this);
  this->initSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Init"), &RobotControllerPlugin::OnInitMsg, this);
  this->srguiSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Request"), &RobotControllerPlugin::OnRequestMsg, this);
  this->statusSub = this->node->Subscribe(std::string("~/SceneReconstruction/GUI/Availability/Request/RobotController"), &RobotControllerPlugin::OnStatusMsg, this);
  this->srguiPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/RobotController/Response"));
  this->srguiPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Response"));
  this->statusPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));

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
        c.pose.pos.x = robot.pos_x() + this->position_x_offset;
        c.pose.pos.y = robot.pos_y() + this->position_y_offset;
        if(robot.has_pos_z())
          c.pose.pos.z = robot.pos_z() + this->position_z_offset;
        else {
          c.pose.pos.z = 0.0 + this->position_z_offset;
        }

        if(robot.has_ori_w())
          c.pose.rot.w = robot.ori_w() + this->orientation_w_offset;
        else
          c.pose.rot.w = 0.0 + this->orientation_w_offset;

        if(robot.has_ori_x())
          c.pose.rot.x = robot.ori_x() + this->orientation_x_offset;
        else
          c.pose.rot.x = 0.0 + this->orientation_x_offset;

        if(robot.has_ori_y())
          c.pose.rot.y = robot.ori_y() + this->orientation_y_offset;
        else
          c.pose.rot.y = 0.0 + this->orientation_y_offset;

        if(robot.has_ori_z())
          c.pose.rot.z = robot.ori_z() + this->orientation_z_offset;
        else
          c.pose.rot.z = 0.0 + this->orientation_z_offset;
        

        this->robotControlList.push_back(c);
      }
    }
    else {
      gzwarn << "message of unknown type\n";
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

  pose.pos.x = _msg->pos_x() + this->position_x_offset;
  pose.pos.y = _msg->pos_y() + this->position_y_offset;
  if(_msg->has_pos_z())
    pose.pos.z = _msg->pos_z() + this->position_z_offset;
  else {
    pose.pos.z = 0.0 + this->position_z_offset;
  }

  if(_msg->has_ori_w())
    pose.rot.w = _msg->ori_w() + this->orientation_w_offset;
  else
    pose.rot.w = 0.0 + this->orientation_w_offset;

  if(_msg->has_ori_x())
    pose.rot.x = _msg->ori_x() + this->orientation_x_offset;
  else
    pose.rot.x = 0.0 + this->orientation_x_offset;

  if(_msg->has_ori_y())
    pose.rot.y = _msg->ori_y() + this->orientation_y_offset;
  else
    pose.rot.y = 0.0 + this->orientation_y_offset;

  if(_msg->has_ori_z())
    pose.rot.z = _msg->ori_z() + this->orientation_z_offset;
  else
    pose.rot.z = 0.0 + this->orientation_z_offset;
  
  this->model->SetWorldPose(pose);
}

void RobotControllerPlugin::OnInitMsg(ConstSceneRobotControllerPtr &_msg) {
  std::map<std::string,double> positions;
  int rn, ra;
  rn = _msg->robot_name_size();
  ra = _msg->robot_angle_size();

  if(rn == ra) {
    for(int i=0; i<rn; i++) {
      jointiter = jointdata.find(_msg->robot_name(i));
      if(jointiter != jointdata.end()) {
        positions[jointiter->second.simulator_name] = _msg->robot_angle(i) + jointiter->second.offset;
      }
    }

    jointiter = jointdata.end();
    this->model->SetJointPositions(positions);
  }
  else {
    gzerr << "not all joints have name and angle set\n";
  }

  math::Pose pose;

  pose.pos.x = _msg->pos_x() + this->position_x_offset;
  pose.pos.y = _msg->pos_y() + this->position_y_offset;
  if(_msg->has_pos_z())
    pose.pos.z = _msg->pos_z() + this->position_z_offset;
  else {
    pose.pos.z = 0.0 + this->position_z_offset;
  }

  if(_msg->has_ori_w())
    pose.rot.w = _msg->ori_w() + this->orientation_w_offset;
  else
    pose.rot.w = 0.0 + this->orientation_w_offset;

  if(_msg->has_ori_x())
    pose.rot.x = _msg->ori_x() + this->orientation_x_offset;
  else
    pose.rot.x = 0.0 + this->orientation_x_offset;

  if(_msg->has_ori_y())
    pose.rot.y = _msg->ori_y() + this->orientation_y_offset;
  else
    pose.rot.y = 0.0 + this->orientation_y_offset;

  if(_msg->has_ori_z())
    pose.rot.z = _msg->ori_z() + this->orientation_z_offset;
  else
    pose.rot.z = 0.0 + this->orientation_z_offset;
  
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
    src.set_pos_x(pose.pos.x - this->position_x_offset);
    src.set_pos_y(pose.pos.y - this->position_y_offset);
    src.set_pos_z(pose.pos.z - this->position_z_offset);
    src.set_ori_w(pose.rot.w - this->orientation_w_offset);
    src.set_ori_x(pose.rot.x - this->orientation_x_offset);
    src.set_ori_y(pose.rot.y - this->orientation_y_offset);
    src.set_ori_z(pose.rot.z - this->orientation_z_offset);

    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    this->srguiPub->Publish(response);
  }
  else if(_msg->request() == "get_offset") {
    msgs::Pose offset;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response("success");
    response.set_type(offset.GetTypeName());

    msgs::Vector3d *pos = offset.mutable_position();
    pos->set_x(this->position_x_offset);
    pos->set_y(this->position_y_offset);
    pos->set_z(this->position_z_offset);

    msgs::Quaternion *ori = offset.mutable_orientation();
    ori->set_x(this->orientation_x_offset);
    ori->set_y(this->orientation_y_offset);
    ori->set_z(this->orientation_z_offset);
    ori->set_w(this->orientation_w_offset);

    std::string *serializedData = response.mutable_serialized_data();
    offset.SerializeToString(serializedData);

    this->offsetPub->Publish(response);
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
