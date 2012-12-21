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
  this->object_count = 0;

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
                object_list[name].object = model;
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
  this->bufferPub = this->node->Advertise<msgs::Message_V>(std::string("~/SceneReconstruction/GUI/Buffer"));
  this->drawingPub = this->node->Advertise<msgs::Drawing>(std::string("~/draw"));

  this->objectSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Object"), &ObjectInstantiatorPlugin::OnSceneObjectMsg, this);
  this->bufferSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/BufferObject"), &ObjectInstantiatorPlugin::OnBufferObjectMsg, this);
  this->requestSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Request"), &ObjectInstantiatorPlugin::OnRequestMsg, this);
}

void ObjectInstantiatorPlugin::OnRequestMsg(ConstRequestPtr &_msg) {
  // TODO: reimplement with queue
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
  else if(_msg->request() == "update_object_buffer") {
    msgs::Message_V buffer;
    fill_buffer_msg(buffer);
    bufferPub->Publish(buffer);
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
  unsigned int obs = this->object_buffer.size();

  this->ProcessSceneObjectMsgs();
  this->UpdateObjects(now);

  if(this->object_buffer.size() != obs) {
    msgs::Message_V buffer;
    fill_buffer_msg(buffer);
    bufferPub->Publish(buffer);
  }
}

void ObjectInstantiatorPlugin::UpdateObjects(common::Time now) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  if(now >= this->next_buffer) {
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

void ObjectInstantiatorPlugin::OnSceneObjectMsg(ConstSceneObject_VPtr &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->objectMsgs.push_back(*_msg);
}

void ObjectInstantiatorPlugin::OnBufferObjectMsg(ConstBufferObjectsPtr &_msg) {
  if(_msg->timestamp() < 0.0) {
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
    int os = _msg->object_size();
    for(int i=0; i<os; i++) {
      msgs::Drawing drw;
      drw.set_name(_msg->object(i).object());
      physics::ModelPtr mdl = world->GetModel(_msg->object(i).object());
      if(_msg->object(i).visible() && mdl) {
        drw.set_visible(true);
        drw.mutable_pose()->CopyFrom(_msg->object(i).pose());
        drw.set_material("Gazebo/BlueGlow");
        drw.set_mode(msgs::Drawing::LINE_LIST);
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
        math::Box bb = mdl->GetBoundingBox();
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

  /*
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
  */
}

void ObjectInstantiatorPlugin::ProcessSceneObjectMsgs() {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  std::list<msgs::SceneObject_V>::iterator _msg;

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
