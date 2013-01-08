#include <iostream>

#include "common/SystemPaths.hh"
#include "transport/transport.h"
#include "physics/World.hh"
#include "physics/Model.hh"

#include "ObjectInstantiatorPlugin.hh"

using namespace gazebo;

ObjectInstantiatorPlugin::ObjectInstantiatorPlugin() : WorldPlugin() 
{
  this->receiveMutex = new boost::mutex();
}

void ObjectInstantiatorPlugin::Init() {
  this->next_buffer = common::Time(world->GetSimTime());
  out_of_sight.pos.z = -10.0;
}

void ObjectInstantiatorPlugin::Reset() {
  this->next_buffer = common::Time(world->GetSimTime());
  this->objectMsgs.clear();
  this->object_buffer.clear();

  std::map<std::string, SceneObject>::iterator iter;
  for(iter = object_list.begin(); iter != object_list.end(); iter++) {
    iter->second.pose = out_of_sight;
    iter->second.visible = false;
    iter->second.buffertime = common::Time(0.0);
    iter->second.query = "";
  }
}

void ObjectInstantiatorPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->receiveMutex = new boost::mutex();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  this->position_offset = math::Vector3(0.0, 0.0, 0.0);
  __available = false;

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

  sdf::ElementPtr objectElem;
  if (_sdf->HasElement("object")) {
    objectElem = _sdf->GetElement("object");
    std::string object;
    while(objectElem) {
      objectElem->GetValue()->Get(object);
      std::string name;
      std::string model;

      if(_sdf->HasElement("name_"+object)) {
        if(!_sdf->GetElement("name_"+object)->GetValue()->Get(name)) {
      	  gzerr << "<name_" << object << "> is not a string, leaving out this <object>\n";
        }
        else {
          if(_sdf->HasElement("model_"+object)) {
            if(!_sdf->GetElement("model_"+object)->GetValue()->Get(model)) {
          	  gzerr << "<model_" << object << "> is not a string, leaving out this <object>\n";
            }
            else {
              physics::ModelPtr m = this->world->GetModel(model);
              if(!m) {
            	  gzerr << "model named \"" << model << "\" not found, leaving out this <object>\n";
              }
              else {
                object_list[name].object = name;
                object_list[name].model = m;
                object_names[model] = name;
              }
            }
          }
          else {
        	  gzerr << "missing required element <model_" << object << ">, leaving out this <object>\n";
          }
        }
      }
      else {
      	gzerr << "missing required element <name_" << object << ">, leaving out this <object>\n";
      }
      objectElem = objectElem->GetNextElement("object");
    }
  }

  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
          boost::bind(&ObjectInstantiatorPlugin::OnUpdate, this)));

  this->srguiPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/ObjectInstantiator/Response"));
  this->objectPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Response"));
  this->framePub = this->node->Advertise<msgs::Request>(std::string("~/SceneReconstruction/Framework/Request"));
  this->requestPub = this->node->Advertise<msgs::Request>(std::string("~/request"));
  this->statusPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));
  this->drawingPub = this->node->Advertise<msgs::Drawing>(std::string("~/draw"));

  this->objectSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Object"), &ObjectInstantiatorPlugin::OnSceneObjectMsg, this);
  this->bufferSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/BufferObject"), &ObjectInstantiatorPlugin::OnObjectMsg, this);
  this->requestSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Request"), &ObjectInstantiatorPlugin::OnRequestMsg, this);
}

void ObjectInstantiatorPlugin::OnRequestMsg(ConstRequestPtr &_msg) {
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");

  if(_msg->request() == "available") {
    __available = true;
  }
  else if(_msg->request() == "object_list") {
    msgs::GzString_V src;
    response.set_type(src.GetTypeName());
    fill_list_msg(src);
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    this->srguiPub->Publish(response);
  }
  else if(_msg->request() == "object_data") {
    response.set_response("success");
    msgs::SceneObject scob;
    response.set_type(scob.GetTypeName());
    if(_msg->has_data() && fill_object_msg(_msg->data(), scob)) {
      msgs::Request request;
      request.set_request(_msg->request());
      request.set_id(_msg->id());
      request.set_data(scob.query());
      std::string *serializedData = response.mutable_serialized_data();
      scob.SerializeToString(serializedData);

      this->framePub->Publish(request);
      this->srguiPub->Publish(response);
    }
    else {
      response.set_response("failure");
      msgs::GzString src;
      response.set_type(src.GetTypeName());
      src.set_data(_msg->data()+" not an object known to the objectinstantiator");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);

      this->srguiPub->Publish(response);
    }
  }
  else if(_msg->request() == "get_frame") {
    std::map<std::string, std::string>::iterator name = object_names.find(_msg->data());
    if(name != object_names.end()) {
      std::map<std::string, SceneObject>::iterator it =  object_list.find(name->second);
      if(it != object_list.end()) {
        msgs::GzString src;
        response.set_type(src.GetTypeName());

        src.set_data(it->second.frame);

        std::string *serializedData = response.mutable_serialized_data();
        src.SerializeToString(serializedData);
      }
    }
    else {
      response.set_response("object "+_msg->data()+" unknown");
    }
    this->objectPub->Publish(response);
  }
  else {
    response.set_response("unknown");
    msgs::GzString src;
    response.set_type(src.GetTypeName());
    src.set_data("the given request is unknown to the objectinstantiator");
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);
    this->srguiPub->Publish(response);
  }
}


void ObjectInstantiatorPlugin::OnUpdate() {
  if(!__available) {
    msgs::Response response;
    response.set_id(-1);
    response.set_request("status");
    response.set_response("ObjectInstantiator");
    this->statusPub->Publish(response);
  }

  if(this->world->IsPaused())
    return;

  common::Time now = common::Time(world->GetSimTime());
  this->ProcessSceneObjectMsgs();
  this->UpdateObjects(now);
}

void ObjectInstantiatorPlugin::UpdateObjects(common::Time now) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  if(now >= this->next_buffer) {
    this->next_buffer = now + common::Time(1);
    std::list<SceneObject>::iterator it;
    std::list<SceneObject> new_object_buffer;
    for(it = object_buffer.begin(); it != object_buffer.end(); it++) {
      if(it->buffertime <= now) {
        update_object(*it);
      }
      else {
        if(it->buffertime < this->next_buffer)
          this->next_buffer = it->buffertime;
        new_object_buffer.push_back(*it);
      }
    }
    object_buffer = new_object_buffer;
    object_buffer.sort();
  }
}

void ObjectInstantiatorPlugin::update_object(SceneObject obj) {
  bool visible = object_list[obj.object].visible;
  object_list[obj.object].visible = obj.visible;
  object_list[obj.object].pose = obj.pose;

  //TODO: proper fix for Object Pose
  if(obj.object.find("Object") != std::string::npos) {
    object_list[obj.object].pose.rot.w = 1.0;
    object_list[obj.object].pose.rot.x = 0.0;
    object_list[obj.object].pose.rot.y = 0.0;
    object_list[obj.object].pose.rot.z = 0.0;
  }

  object_list[obj.object].buffertime = obj.buffertime;
  object_list[obj.object].query = obj.query;
  object_list[obj.object].frame = obj.frame;

  if(visible && !obj.visible) {
    // object became invisible, so move it out of sight (10 meters below the floor)
    if(object_list[obj.object].model) {
      object_list[obj.object].model->SetWorldPose(out_of_sight);
    }
  }
  else if (obj.visible) {
    // object visible, so update pose
    if(object_list[obj.object].model) {
      object_list[obj.object].model->SetWorldPose(object_list[obj.object].pose);
    }

  }
}

bool ObjectInstantiatorPlugin::fill_object_msg(std::string name, msgs::SceneObject &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::map<std::string, SceneObject>::iterator it =  object_list.find(name);
  if(it != object_list.end()) {
    _msg.set_object(it->second.object);
    _msg.set_visible(it->second.visible);
    msgs::Pose *p = _msg.mutable_pose();
    math::Pose pose;
    if(it->second.model)
      p->CopyFrom(msgs::Convert(it->second.model->GetWorldPose()));
    else
      p->CopyFrom(msgs::Convert(pose));
    _msg.set_time(it->second.buffertime.Double());
    _msg.set_query(it->second.query);
    _msg.set_frame(it->second.frame);
    return true;
  }
  
  return false;
}

void ObjectInstantiatorPlugin::fill_list_msg(msgs::GzString_V &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::map<std::string, SceneObject>::iterator it;
  for(it = object_list.begin(); it != object_list.end(); it++) {
    _msg.add_data(it->first);
  }
}

/*
void ObjectInstantiatorPlugin::fill_buffer_msg(msgs::Message_V &_msg) {
  std::list<SceneObject>::iterator it;
  common::Time time(0.0);
  msgs::BufferObjects obj;
  _msg.set_msgtype(obj.GetTypeName());
  for(it = object_buffer.begin(); it != object_buffer.end(); it++) {
    if(time != it->buffertime) {
      if(time != 0.0) {
        std::string *msg = _msg.add_msgsdata();
        obj.SerializeToString(msg);
      }
      time = it->buffertime;
      obj.clear_object();
      double timestamp;
      timestamp  = time.sec*1000.0;
      timestamp += time.nsec/1000000.0;
      obj.set_timestamp(timestamp);
    }

    msgs::SceneObject *object = obj.add_object();
    object->set_object(it->object);
    object->set_visible(it->visible);
    msgs::Pose *pose = object->mutable_pose();
    msgs::Set(pose, it->pose);
    object->set_query(it->query);
  }

  if(time != 0.0) {
    std::string *msg = _msg.add_msgsdata();
    obj.SerializeToString(msg);
  }
}
*/

void ObjectInstantiatorPlugin::OnSceneObjectMsg(ConstMessage_VPtr &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->objectMsgs.push_back(*_msg);
}

void ObjectInstantiatorPlugin::OnObjectMsg(ConstSceneObjectPtr &_msg) {
  if(_msg->time() < 0.0) {
    std::map<std::string, SceneObject>::iterator iter;
    for(iter = object_list.begin(); iter != object_list.end(); iter++) {
      msgs::Drawing drw;
      drw.set_name(iter->first);
      drw.set_visible(false);
      drawingPub->Publish(drw);
    }
  }
  else {
    // create bounding box drawings
    msgs::Drawing drw;
    drw.set_name(_msg->object());
    physics::ModelPtr mdl;
    std::map<std::string, SceneObject>::iterator it =  object_list.find(_msg->object());
    if(it != object_list.end()) {
      mdl = it->second.model;
    }

    if(_msg->visible() && mdl) {
      drw.set_visible(true);
      math::Pose p = msgs::Convert(_msg->pose());
      p.pos += position_offset;

      // TODO: proper fix for Object Pose 
      if(_msg->object().find("Object") != std::string::npos) {
        p.rot.w = 1.0;
        p.rot.x = 0.0;
        p.rot.y = 0.0;
        p.rot.z = 0.0;
      }

      drw.mutable_pose()->CopyFrom(msgs::Convert(p));
      drw.set_material("SceneReconstruction/Object");
      drw.set_mode(msgs::Drawing::LINE_LIST);

      // add points for all 12 lines of the bounding box
      msgs::Drawing::Point *p11 = drw.add_point();
      msgs::Drawing::Point *p12 = drw.add_point();
      msgs::Drawing::Point *p21 = drw.add_point();
      msgs::Drawing::Point *p22 = drw.add_point();
      msgs::Drawing::Point *p31 = drw.add_point();
      msgs::Drawing::Point *p32 = drw.add_point();
      msgs::Drawing::Point *p41 = drw.add_point();
      msgs::Drawing::Point *p42 = drw.add_point();
      msgs::Drawing::Point *p51 = drw.add_point();
      msgs::Drawing::Point *p52 = drw.add_point();
      msgs::Drawing::Point *p61 = drw.add_point();
      msgs::Drawing::Point *p62 = drw.add_point();
      msgs::Drawing::Point *p71 = drw.add_point();
      msgs::Drawing::Point *p72 = drw.add_point();
      msgs::Drawing::Point *p81 = drw.add_point();
      msgs::Drawing::Point *p82 = drw.add_point();
      msgs::Drawing::Point *p91 = drw.add_point();
      msgs::Drawing::Point *p92 = drw.add_point();
      msgs::Drawing::Point *pA1 = drw.add_point();
      msgs::Drawing::Point *pA2 = drw.add_point();
      msgs::Drawing::Point *pB1 = drw.add_point();
      msgs::Drawing::Point *pB2 = drw.add_point();
      msgs::Drawing::Point *pC1 = drw.add_point();
      msgs::Drawing::Point *pC2 = drw.add_point();

      // get bounding box and "move" it to 0,0,0
      math::Box bb = mdl->GetBoundingBox();
      bb.max = bb.max - mdl->GetWorldPose().pos;
      bb.min = bb.min - mdl->GetWorldPose().pos;

      // set the respective coordinates for the points
      p11->mutable_position()->set_x(bb.max.x);
      p11->mutable_position()->set_y(bb.max.y);
      p11->mutable_position()->set_z(bb.max.z);
      p12->mutable_position()->set_x(bb.max.x);
      p12->mutable_position()->set_y(bb.min.y);
      p12->mutable_position()->set_z(bb.max.z);

      p21->mutable_position()->set_x(bb.max.x);
      p21->mutable_position()->set_y(bb.max.y);
      p21->mutable_position()->set_z(bb.max.z);
      p22->mutable_position()->set_x(bb.min.x);
      p22->mutable_position()->set_y(bb.max.y);
      p22->mutable_position()->set_z(bb.max.z);

      p31->mutable_position()->set_x(bb.max.x);
      p31->mutable_position()->set_y(bb.min.y);
      p31->mutable_position()->set_z(bb.max.z);
      p32->mutable_position()->set_x(bb.min.x);
      p32->mutable_position()->set_y(bb.min.y);
      p32->mutable_position()->set_z(bb.max.z);

      p41->mutable_position()->set_x(bb.min.x);
      p41->mutable_position()->set_y(bb.max.y);
      p41->mutable_position()->set_z(bb.max.z);
      p42->mutable_position()->set_x(bb.min.x);
      p42->mutable_position()->set_y(bb.min.y);
      p42->mutable_position()->set_z(bb.max.z);

      p51->mutable_position()->set_x(bb.max.x);
      p51->mutable_position()->set_y(bb.max.y);
      p51->mutable_position()->set_z(bb.min.z);
      p52->mutable_position()->set_x(bb.max.x);
      p52->mutable_position()->set_y(bb.min.y);
      p52->mutable_position()->set_z(bb.min.z);

      p61->mutable_position()->set_x(bb.max.x);
      p61->mutable_position()->set_y(bb.max.y);
      p61->mutable_position()->set_z(bb.min.z);
      p62->mutable_position()->set_x(bb.min.x);
      p62->mutable_position()->set_y(bb.max.y);
      p62->mutable_position()->set_z(bb.min.z);

      p71->mutable_position()->set_x(bb.max.x);
      p71->mutable_position()->set_y(bb.min.y);
      p71->mutable_position()->set_z(bb.min.z);
      p72->mutable_position()->set_x(bb.min.x);
      p72->mutable_position()->set_y(bb.min.y);
      p72->mutable_position()->set_z(bb.min.z);

      p81->mutable_position()->set_x(bb.min.x);
      p81->mutable_position()->set_y(bb.max.y);
      p81->mutable_position()->set_z(bb.min.z);
      p82->mutable_position()->set_x(bb.min.x);
      p82->mutable_position()->set_y(bb.min.y);
      p82->mutable_position()->set_z(bb.min.z);

      p91->mutable_position()->set_x(bb.max.x);
      p91->mutable_position()->set_y(bb.max.y);
      p91->mutable_position()->set_z(bb.max.z);
      p92->mutable_position()->set_x(bb.max.x);
      p92->mutable_position()->set_y(bb.max.y);
      p92->mutable_position()->set_z(bb.min.z);

      pA1->mutable_position()->set_x(bb.min.x);
      pA1->mutable_position()->set_y(bb.max.y);
      pA1->mutable_position()->set_z(bb.max.z);
      pA2->mutable_position()->set_x(bb.min.x);
      pA2->mutable_position()->set_y(bb.max.y);
      pA2->mutable_position()->set_z(bb.min.z);

      pB1->mutable_position()->set_x(bb.max.x);
      pB1->mutable_position()->set_y(bb.min.y);
      pB1->mutable_position()->set_z(bb.max.z);
      pB2->mutable_position()->set_x(bb.max.x);
      pB2->mutable_position()->set_y(bb.min.y);
      pB2->mutable_position()->set_z(bb.min.z);

      pC1->mutable_position()->set_x(bb.min.x);
      pC1->mutable_position()->set_y(bb.min.y);
      pC1->mutable_position()->set_z(bb.max.z);
      pC2->mutable_position()->set_x(bb.min.x);
      pC2->mutable_position()->set_y(bb.min.y);
      pC2->mutable_position()->set_z(bb.min.z);
    }
    else
      drw.set_visible(false);

    drawingPub->Publish(drw);
  }
}

void ObjectInstantiatorPlugin::ProcessSceneObjectMsgs() {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  std::list<msgs::Message_V>::iterator _msg;

  for (_msg = this->objectMsgs.begin(); _msg != this->objectMsgs.end(); ++_msg) {
    // add new objects
    msgs::SceneObject obj;
    if(obj.GetTypeName() == _msg->msgtype()) {
      int o = _msg->msgsdata_size();
      for(int n=0; n<o; n++) {
        obj.ParseFromString(_msg->msgsdata(n));
        SceneObject so;
        so.object = obj.object();
        so.pose = msgs::Convert(obj.pose());
        so.pose.pos += position_offset;
        so.visible = obj.visible();
        so.buffertime = common::Time(obj.time());
        so.query = obj.query();
        so.frame = obj.frame();

        if(so.buffertime <= common::Time(world->GetSimTime())) {
          update_object(so);
        }
        else {
          if(so.buffertime <= this->next_buffer)
            this->next_buffer = so.buffertime;
          object_buffer.push_back(so);
        }
      }
    }
  }

  object_buffer.sort();
  objectMsgs.clear();
}
