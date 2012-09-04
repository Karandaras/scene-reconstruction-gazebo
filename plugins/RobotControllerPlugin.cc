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

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->jointSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Joint"), &RobotControllerPlugin::OnSceneJointMsg, this);
  this->changeJointSub = this->node->Subscribe(std::string("~/SceneReconstruction/RobotController/Joint/setup"), &RobotControllerPlugin::OnSceneChangeMsg, this);
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

  msgs::Response response;
  response.set_id(-1);
  response.set_request("status");
  response.set_response("RobotController");
  this->statusPub->Publish(response);
}

/////////////////////////////////////////////////
void RobotControllerPlugin::Init()
{
}
/////////////////////////////////////////////////
void RobotControllerPlugin::OnSceneChangeMsg(ConstSceneRobotControllerPtr &_msg) {
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
      jointiter->second.simulator_name = _msg->simulator_name(i);
      jointiter->second.offset = _msg->offset(i);
      jointiter->second.simulator_angle = _msg->simulator_angle(i);
      jointiter->second.robot_angle = _msg->robot_angle(i);
      positions[jointiter->second.simulator_name] = jointiter->second.simulator_angle;
    }

    jointiter = jointdata.end();
    this->model->SetJointPositions(positions);
  }
  else {
    gzerr << "not all fields set for all joints\n";
  }
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnSceneJointMsg(ConstSceneJointPtr &_msg) {
  std::map<std::string,double> positions;

  int joints = _msg->joint_size();
  if(joints <= (int)this->model->GetJointCount()) {
    if(joints == _msg->angle_size()) {
      for(int i=0; i<joints; i++) {
        this->jointiter = this->jointdata.find(_msg->joint(i));
        if(this->jointiter != jointdata.end()) {
          positions[this->jointiter->second.simulator_name] = this->jointiter->second.offset+_msg->angle(i);
 	        this->jointiter->second.simulator_angle = this->jointiter->second.offset+_msg->angle(i);
          this->jointiter->second.robot_angle = _msg->angle(i);
        }
      }
      this->jointiter = this->jointdata.end();

      this->model->SetJointPositions(positions);
    } else {
      gzerr << "number of joints differs from number of angles\n";
    }
  } else {
    gzerr << "message tries to alter more joints then available\n";
  }
  
  positions.clear();
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

