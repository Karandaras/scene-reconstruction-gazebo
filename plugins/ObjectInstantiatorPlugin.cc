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
  update_object_buffer = false;
}

void ObjectInstantiatorPlugin::Reset() {
  this->next_buffer = common::Time(world->GetSimTime());
  this->objectMsgs.clear();
  this->object_buffer.clear();
  this->object_count = 0;
  update_object_buffer = false;

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
  this->object_count = 0;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

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
                object_list[name].object = model;
                object_list[name].model = m;
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
  this->bufferPub = this->node->Advertise<msgs::Message_V>(std::string("~/SceneReconstruction/GUI/Buffer"));

  std::map<std::string, SceneObject>::iterator obj; 
  for(obj = object_list.begin(); obj != object_list.end(); obj++) {
    msgs::Request *req;
    req = msgs::CreateRequest("set_transparency", obj->second.object+"_clone");
    req->set_dbl_data(0.75);
    requestPub->Publish(*req);
  }

  msgs::Response response;
  response.set_id(-1);
  response.set_request("status");
  response.set_response("ObjectInstantiator");
  this->statusPub->Publish(response);

  this->objectSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Object"), &ObjectInstantiatorPlugin::OnSceneObjectMsg, this);
  this->bufferSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/BufferObject"), &ObjectInstantiatorPlugin::OnBufferObjectMsg, this);
  this->requestSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Request"), &ObjectInstantiatorPlugin::OnRequestMsg, this);
  this->statusSub = this->node->Subscribe(std::string("~/SceneReconstruction/GUI/Availability/Request/ObjectInstantiator"), &ObjectInstantiatorPlugin::OnStatusMsg, this);
}

void ObjectInstantiatorPlugin::OnRequestMsg(ConstRequestPtr &_msg) {
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");

  if(_msg->request() == "object_list") {
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
  else if(_msg->request() == "object_repository") {
    msgs::GzString_V src;
    response.set_type(src.GetTypeName());
    fill_repository_msg(src);
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    this->srguiPub->Publish(response);
  }
  else if(_msg->request() == "get_frame") {
    std::map<std::string, SceneObject>::iterator it =  object_list.find(_msg->data());
    if(it != object_list.end()) {
      msgs::GzString src;
      response.set_type(src.GetTypeName());

      src.set_data(it->second.frame);

      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
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

void ObjectInstantiatorPlugin::OnStatusMsg(ConstRequestPtr &_msg) {
  if(_msg->request() == "status") {
    msgs::Response response;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response("ObjectInstantiator");
    this->statusPub->Publish(response);
  }
}

void ObjectInstantiatorPlugin::OnUpdate() {
  if(this->world->IsPaused())
    return;

  common::Time now = common::Time(world->GetSimTime());
  this->ProcessSceneObjectMsgs();
  this->UpdateObjects(now);

  if(update_object_buffer) {
    update_object_buffer = false;
    msgs::Message_V buffer;
    fill_buffer_msg(buffer);
    bufferPub->Publish(buffer);
  }
}

void ObjectInstantiatorPlugin::UpdateObjects(common::Time now) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  if(now >= this->next_buffer) {
    update_object_buffer = true;
    this->next_buffer = now + common::Time(1);
    std::list<SceneObject>::iterator it;
    std::list<SceneObject> new_object_buffer;
    for(it = object_buffer.begin(); it != object_buffer.end(); it++) {
      if(it->buffertime < now) {
        update_object(*it);
      }
      else if(it->buffertime < this->next_buffer) {
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
    if(object_list[obj.object].model)
      object_list[obj.object].model->SetWorldPose(out_of_sight);
  }
  else if (obj.visible) {
    // object visible, so update pose
    if(object_list[obj.object].model)
      object_list[obj.object].model->SetWorldPose(object_list[obj.object].pose);
  }
}

bool ObjectInstantiatorPlugin::fill_object_msg(std::string name, msgs::SceneObject &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::map<std::string, SceneObject>::iterator it =  object_list.find(name);
  if(it != object_list.end()) {
    _msg.set_object(it->second.object);
    const math::Pose pose = this->world->GetModel(it->second.object)->GetWorldPose();
    _msg.set_visible(it->second.visible);
    msgs::Pose *p = _msg.mutable_pose();
    p->CopyFrom(msgs::Convert(pose));
    _msg.set_time(it->second.buffertime.Double());
    _msg.set_query(it->second.query);
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

void ObjectInstantiatorPlugin::fill_repository_msg(msgs::GzString_V &_msg) {
  std::map<std::string, SceneObject>::iterator it;
  for(it = object_list.begin(); it != object_list.end(); it++) {
    _msg.add_data(it->first);
  }
}

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
      obj.set_timestamp(time.Double());
    }

    msgs::SceneObject *object = obj.add_object();
    object->set_object(it->object);
    object->set_visible(it->visible);
    object->set_model(it->model->GetName());
    msgs::Pose *pose = object->mutable_pose();
    msgs::Set(pose, it->pose);
    object->set_query(it->query);
  }
}

void ObjectInstantiatorPlugin::OnSceneObjectMsg(ConstSceneObject_VPtr &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->objectMsgs.push_back(*_msg);
}

void ObjectInstantiatorPlugin::OnBufferObjectMsg(ConstBufferObjectsPtr &_msg) {
  if(_msg->timestamp() < 0.0) {
    std::map<std::string, SceneObject>::iterator iter;
    for(iter = object_list.begin(); iter != object_list.end(); iter++) {
      physics::ModelPtr mdl = world->GetModel(iter->first+"_clone");
      if(mdl) {
        mdl->SetWorldPose(out_of_sight);
      }
    }
  }
  else {
    int o = _msg->object_size();
    for(int i=0; i<o; i++) {
      physics::ModelPtr mdl = world->GetModel(_msg->object(i).object()+"_clone");
      if(mdl) {
        // Move mdl
        if(_msg->object(i).visible())
          mdl->SetWorldPose(msgs::Convert(_msg->object(i).pose()));
        else
          mdl->SetWorldPose(out_of_sight);
      }
    }
  }
}

void ObjectInstantiatorPlugin::ProcessSceneObjectMsgs() {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  std::list<msgs::SceneObject_V>::iterator _msg;
  if(!objectMsgs.empty())
    update_object_buffer = true;

  for (_msg = this->objectMsgs.begin(); _msg != this->objectMsgs.end(); ++_msg) {
   // add new objects
    int o,v,p,t,q,f;
    o = _msg->object_size();
    v = _msg->visible_size();
    p = _msg->pose_size();
    t = _msg->time_size();
    q = _msg->query_size();
    f = _msg->frame_size();
    if(o == v && v == p && p == t && t == q && q == f && f == o) {
      for(int n=0; n<o; n++) {
        SceneObject so;
        so.object = _msg->object(n);
        so.pose = msgs::Convert(_msg->pose(n));
        so.pose.pos += position_offset;
        so.visible = _msg->visible(n);
        so.buffertime = common::Time(_msg->time(n));
        so.query = _msg->query(n);
        so.frame = _msg->frame(n);

        if(so.buffertime <= common::Time(world->GetSimTime())) {
          update_object(so);
        }
        else if(so.buffertime <= this->next_buffer) {
          this->next_buffer = so.buffertime;
          object_buffer.push_back(so);
        }
      }
    }
  }

  object_buffer.sort();
  objectMsgs.clear();
}
