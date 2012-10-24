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

  this->position_offset = math::Vector3(0.0, 0.0, 0.0);

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
        this->position_offset.x = x_offset;
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
        this->position_offset.y = y_offset;
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
        this->position_offset.z = z_offset;
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
      std::string gname, gname2;
      std::string fname;
      std::string offtmp;
      std::string factmp;
      std::string grippertmp;
      double off = 0.0;
      double fac = 1.0;
      bool gripper = false;

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

          if(_sdf->HasElement("factor_"+rcjoint)) {
            if(!_sdf->GetElement("factor_"+rcjoint)->GetValue()->Get(factmp)) {
              gzwarn << "<factor_" << rcjoint << "> not readable, defaulting to 1.0\n";
              fac = 1.0;
            }
            else {
              char* t;
              fac = strtod(factmp.c_str(), &t);
              if(*t != 0) {
                gzwarn << "<factor_" << rcjoint << "> not a double, defaulting to 1.0\n";
                fac = 1.0;
              }  
            }
          } else {
            fac = 1.0;
          }

          if(_sdf->HasElement("gripper_"+rcjoint)) {
            if(!_sdf->GetElement("gripper_"+rcjoint)->GetValue()->Get(grippertmp)) {
              gzwarn << "<gripper_" << rcjoint << "> not readable, defaulting to false\n";
            }
            else {
              std::transform(grippertmp.begin(), grippertmp.end(), grippertmp.begin(), ::tolower);
              if(grippertmp == "true" || grippertmp == "1") {
                gripper = true;
              }
              else if(grippertmp == "false" || grippertmp == "0") {
                gripper = false;
              }
              else {
                gzwarn << "<gripper_" << rcjoint << "> not valid (" << grippertmp << " not true or false), defaulting to false\n";
              }
            }
          }

          if(gripper) {
            if(_sdf->HasElement("gname2_"+rcjoint)) {
              if(!_sdf->GetElement("gname2_"+rcjoint)->GetValue()->Get(gname2)) {
                gzwarn << "<gname2_" << rcjoint << "> not a string, setting gripper to false\n";
                gripper = false;
              }
            } else {
              gzwarn << "<gname2_" << rcjoint << "> not present, setting gripper to false\n";
              gripper = false;
            }
          }

          physics::JointPtr j = _model->GetJoint(gname);
          if(!j) {
            gzerr << "unable to find joint " << gname << "\n";
          } else {
            double simangle = j->GetAngle(0).Radian();

            this->jointdata[fname].simulator_name = gname;
            this->jointdata[fname].robot_name = fname;
            this->jointdata[fname].offset = off;
            this->jointdata[fname].factor = fac;
            this->jointdata[fname].simulator_angle = simangle;
            this->jointdata[fname].robot_angle = simangle-off;
            this->jointdata[fname].gripper = gripper;
            this->jointdata[fname].simulator_name2 = "";
            if(gripper) {
              physics::JointPtr j2 = _model->GetJoint(gname2);
              if(!j2) {
                gzerr << "unable to find joint " << gname2 << ", setting gripper to false\n";
                this->jointdata[fname].gripper = false;                
              } else {
                this->jointdata[fname].simulator_name2 = gname2;
              }
            }
          }
        }
      } else {
        gzerr << "missing required element <gname_" << rcjoint << ">, leaving out this <rcjoint>\n";
      }
      jointElem = jointElem->GetNextElement("rcjoint");
    }
  }

  this->srguiPub = this->node->Advertise<msgs::SceneRobotController>(std::string("~/SceneReconstruction/RobotController/ControllerInfo"));
  this->offsetPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Response"));
  this->statusPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));
  this->bufferPub = this->node->Advertise<msgs::Message_V>(std::string("~/SceneReconstruction/GUI/Buffer"));

  // connect update to worldupdate
  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
          boost::bind(&RobotControllerPlugin::OnUpdate, this)));

  msgs::Response response;
  response.set_id(-1);
  response.set_request("status");
  response.set_response("RobotController");
  this->statusPub->Publish(response);

  this->controlSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/"), &RobotControllerPlugin::OnControlMsg, this);
  this->initSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Init"), &RobotControllerPlugin::OnInitMsg, this);
  this->srguiSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Request"), &RobotControllerPlugin::OnRequestMsg, this);
  this->statusSub = this->node->Subscribe(std::string("~/SceneReconstruction/GUI/Availability/Request/RobotController"), &RobotControllerPlugin::OnStatusMsg, this);
}

/////////////////////////////////////////////////
void RobotControllerPlugin::Init()
{
  this->next_joint_control = common::Time(world->GetSimTime());
  this->next_robot_control = common::Time(world->GetSimTime());
  setpose = false;
  update_position_buffer = false;
  update_joint_buffer = false;
}

void RobotControllerPlugin::Reset()
{
  update_position_buffer = false;
  update_joint_buffer = false;
  this->next_joint_control = common::Time(world->GetSimTime());
  this->next_robot_control = common::Time(world->GetSimTime());
  jointControlList.clear();
  robotControlList.clear();
  controlMsgs.clear();
  if(initMsg)
    InitMsg();
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnUpdate()
{
  if(this->world->IsPaused())
    return;

  common::Time now = common::Time(world->GetSimTime());

  if(now - lastinfo > 1.0) {
    lastinfo = now;
    msgs::SceneRobotController src;
    
    for(jointiter = jointdata.begin(); jointiter != jointdata.end(); jointiter++) {
        src.add_simulator_name(jointiter->second.simulator_name);
        src.add_robot_name(jointiter->second.robot_name);
        src.add_gripper(jointiter->second.gripper);
        src.add_simulator_name2(jointiter->second.simulator_name2);
        src.add_offset(jointiter->second.offset);
        src.add_simulator_angle(jointiter->second.simulator_angle);
	      src.add_robot_angle(jointiter->second.robot_angle);
    }
    math::Pose pose = this->model->GetWorldPose();
    pose.pos -= position_offset;
    src.set_pos_x(pose.pos.x);
    src.set_pos_y(pose.pos.y);
    src.set_pos_z(pose.pos.z);
    src.set_ori_w(pose.rot.w);
    src.set_ori_x(pose.rot.x);
    src.set_ori_y(pose.rot.y);
    src.set_ori_z(pose.rot.z);

    this->srguiPub->Publish(src);
  }

  this->ProcessControlMsgs();
  this->ControlRobot(now);
  this->ControlJoints(now);

  this->model->SetJointPositions(currentjointpositions);
  this->model->SetRelativePose(currentpose);

  if(update_joint_buffer) {
    update_joint_buffer = false;
    msgs::Message_V jointbuffer;
    fill_joint_buffer_msg(jointbuffer);
    bufferPub->Publish(jointbuffer);
  }

  if(update_position_buffer) {
    update_position_buffer = false;
    msgs::Message_V positionbuffer;
    fill_position_buffer_msg(positionbuffer);
    bufferPub->Publish(positionbuffer);
  }
}

void RobotControllerPlugin::ControlJoints(common::Time now) {
  boost::mutex::scoped_lock lock(*this->jointMutex);
  if(now >= this->next_joint_control) {
    update_joint_buffer = true;
    // process JointCommand list
    this->next_joint_control = now + common::Time(1);
    std::list< JointCommand >::iterator it;
    std::list< JointCommand > newJointList;
    for(it = jointControlList.begin(); it != jointControlList.end(); it++) {
      if(it->controltime < now) {
        std::map<std::string,double>::iterator pos;

        for(pos = it->positions.begin(); pos != it->positions.end(); pos++) {
          this->jointiter = this->jointdata.find(pos->first);
          if(this->jointiter != this->jointdata.end()) {
            this->jointiter->second.robot_angle = pos->second;
            this->jointiter->second.simulator_angle = pos->second * this->jointiter->second.factor + this->jointiter->second.offset;
          }
        }
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

    // set current position for all joints
    for(this->jointiter = jointdata.begin(); this->jointiter != jointdata.end(); this->jointiter++) {
      currentjointpositions[this->jointiter->second.simulator_name] = this->jointiter->second.simulator_angle;
      if(this->jointiter->second.gripper)
        currentjointpositions[this->jointiter->second.simulator_name2] = this->jointiter->second.simulator_angle;
    }
  }
}

void RobotControllerPlugin::ControlRobot(common::Time now) {
  boost::mutex::scoped_lock lock(*this->robotMutex);
  if(now >= this->next_robot_control) {
    update_position_buffer = true;
    this->next_robot_control = now + common::Time(1);
    std::list< RobotCommand >::iterator it;
    std::list< RobotCommand > newRobotList;
    for(it = robotControlList.begin(); it != robotControlList.end(); it++) {
      if(it->controltime < now) {
//        this->model->SetWorldPose(it->pose);
          currentpose = it->pose;
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
  unsigned int jcl = jointControlList.size();
  unsigned int rcl = robotControlList.size();
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
                positions[joint.joint(i)] = joint.angle(i);
            }
            
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
          c.pose.pos.z = 0.0;
        }

        if(robot.has_ori_w())
          c.pose.rot.w = robot.ori_w();
        else
          c.pose.rot.w = 0.0;

        if(robot.has_ori_x())
          c.pose.rot.x = robot.ori_x();
        else
          c.pose.rot.x = 0.0;

        if(robot.has_ori_y())
          c.pose.rot.y = robot.ori_y();
        else
          c.pose.rot.y = 0.0;

        if(robot.has_ori_z())
          c.pose.rot.z = robot.ori_z();
        else
          c.pose.rot.z = 0.0;
        
        c.pose.pos += position_offset;

        this->robotControlList.push_back(c);
      }
    }
    else {
      gzwarn << "message of unknown type\n";
    }
  }

  if(jcl != jointControlList.size())
    update_joint_buffer = true;
  if(rcl != robotControlList.size())
    update_position_buffer = true;

  jointControlList.sort();
  robotControlList.sort();
  controlMsgs.clear();
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnInitMsg(ConstSceneRobotControllerPtr &_msg) {
  initMsg = _msg;
  InitMsg();
}


void RobotControllerPlugin::InitMsg() {
  std::map<std::string,double> positions;
  int rn, ra;
  rn = initMsg->robot_name_size();
  ra = initMsg->robot_angle_size();

  if(rn == ra) {
    for(int i=0; i<rn; i++) {
      jointiter = jointdata.find(initMsg->robot_name(i));
      if(jointiter != jointdata.end()) {
        currentjointpositions[jointiter->second.simulator_name] = initMsg->robot_angle(i)*this->jointiter->second.factor + jointiter->second.offset;
        if(jointiter->second.gripper) {
          currentjointpositions[jointiter->second.simulator_name2] = initMsg->robot_angle(i)*this->jointiter->second.factor + jointiter->second.offset;
        }
        jointiter->second.robot_angle = initMsg->robot_angle(i);
        jointiter->second.simulator_angle = initMsg->robot_angle(i)*this->jointiter->second.factor + jointiter->second.offset;

      }
    }

    jointiter = jointdata.end();
  }
  else {
    gzerr << "not all joints have name and angle set\n";
  }

  math::Pose pose;

  pose.pos.x = initMsg->pos_x();
  pose.pos.y = initMsg->pos_y();
  if(initMsg->has_pos_z())
    pose.pos.z = initMsg->pos_z();
  else {
    pose.pos.z = 0.0;
  }

  if(initMsg->has_ori_w())
    pose.rot.w = initMsg->ori_w();
  else
    pose.rot.w = 0.0;

  if(initMsg->has_ori_x())
    pose.rot.x = initMsg->ori_x();
  else
    pose.rot.x = 0.0;

  if(initMsg->has_ori_y())
    pose.rot.y = initMsg->ori_y();
  else
    pose.rot.y = 0.0;

  if(initMsg->has_ori_z())
    pose.rot.z = initMsg->ori_z();
  else
    pose.rot.z = 0.0;
  
  pose.pos += position_offset;

//  this->model->SetWorldPose(pose);
  currentpose = pose;
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
  if(_msg->request() == "get_data") {
    msgs::Vector3d offset;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response(this->model->GetName());
    response.set_type(offset.GetTypeName());

    offset.set_x(this->position_offset.x);
    offset.set_y(this->position_offset.y);
    offset.set_z(this->position_offset.z);

    std::string *serializedData = response.mutable_serialized_data();
    offset.SerializeToString(serializedData);

    this->offsetPub->Publish(response);
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

void RobotControllerPlugin::fill_joint_buffer_msg(msgs::Message_V &_msg) {
  std::list<JointCommand>::iterator it;
  common::Time time(0.0);
  msgs::BufferJoints jnt;
  _msg.set_msgtype(jnt.GetTypeName());
  for(it = jointControlList.begin(); it != jointControlList.end(); it++) {
    if(time != it->controltime) {
      if(time != 0.0) {
        std::string *msg = _msg.add_msgsdata();
        jnt.SerializeToString(msg);
      }
      time = it->controltime;
      jnt.clear_name();
      jnt.clear_angle();
      jnt.set_timestamp(time.Double());
    }

    std::map<std::string, double>::iterator joint;
    for(joint = it->positions.begin(); joint != it->positions.end(); joint++) {
      jnt.add_name(joint->first);
      jnt.add_angle(joint->second);
    }
  }
}

void RobotControllerPlugin::fill_position_buffer_msg(msgs::Message_V &_msg) {
  std::list<RobotCommand>::iterator it;
  msgs::BufferPosition pos;
  _msg.set_msgtype(pos.GetTypeName());
  for(it = robotControlList.begin(); it != robotControlList.end(); it++) {
    pos.set_timestamp(it->controltime.Double());
    msgs::Pose *pose = pos.mutable_position();
    msgs::Set(pose, it->pose);

    std::string *msg = _msg.add_msgsdata();
    pos.SerializeToString(msg);
  }
}


