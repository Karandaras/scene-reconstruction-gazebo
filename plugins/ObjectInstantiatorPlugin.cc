#include <iostream>

#include "common/SystemPaths.hh"
#include "transport/transport.h"
#include "physics/World.hh"
#include "physics/Model.hh"

#include "ObjectInstantiatorPlugin.hh"

using namespace gazebo;

ObjectInstantiatorPlugin::ObjectInstantiatorPlugin() : WorldPlugin() 
{
  this->object_lifetime = common::Time(0.5);
}

void ObjectInstantiatorPlugin::Init() {
  this->next_expire = common::Time::GetWallTime();
}

void ObjectInstantiatorPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->object_count = 0;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  this->objectSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Object"), &ObjectInstantiatorPlugin::OnSceneObjectMsg, this);
  this->srguiPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/ObjectInstantiator/Response"));
  this->framePub = this->node->Advertise<msgs::Request>(std::string("~/SceneReconstruction/ObjectInstantiator/Request"));

  sdf::ElementPtr settingsElem;
  if (_sdf->HasElement("settings")) {
    settingsElem = _sdf->GetElement("settings");
    if(settingsElem->HasElement("object_lifetime")) {
      double lifetime;
      if(!settingsElem->GetElement("object_lifetime")->GetValue()->Get(lifetime)) {
    	  gzerr << "<object_lifetime> is not a double, defaulting to 0.5 seconds\n";
      }
      else {
        this->object_lifetime = common::Time(lifetime);
      }
    }
  }

  sdf::ElementPtr objectElem;
  if (_sdf->HasElement("object")) {
    objectElem = _sdf->GetElement("object");
    while(objectElem) {
      std::string name;
      std::string data;

      if(objectElem->HasElement("name")) {
        if(!objectElem->GetElement("name")->GetValue()->Get(name)) {
      	  gzerr << "<name> is not a string, leaving out this <object>\n";
        }
        else {
          if(objectElem->HasElement("filename")) {
            std::string sdf_filename;
            if(!objectElem->GetElement("filename")->GetValue()->Get(sdf_filename)) {
          	  gzerr << "<filename> is not a string, leaving out this <object>\n";
            }
            
            std::string _filename = common::SystemPaths::Instance()->FindFileWithGazeboPaths(sdf_filename);
            std::ifstream ifile(_filename.c_str());

            if(!ifile) {              
          	  gzerr << "\"" << filename << "\" does not exist, leaving out this <object>\n";              
            }
            else {
              std::ostringstream _data;
              std::string line;
              while(!ifile.eof()) {
                getline(ifile,line);
                _data << line;
              }
              data = _data.str();

              objects[name] = data;
              ifile.close();
            }
          }
          else if(objectElem->HasElement("data")) {
            if(!objectElem->GetElement("data")->GetValue()->Get(data)) {
          	  gzerr << "<data> is not a string, leaving out this <object>\n";
            }
            else {
              objects[name] = data;
            }
          }
          else {
        	  gzerr << "neither <filename> nor <data> available, leaving out this <object>\n";
          }
        }
      }
      else {
      	gzerr << "missing required element <name>\n";
      }
      objectElem = objectElem->GetNextElement("object");
    }
  }
}

void ObjectInstantiatorPlugin::OnRequestMsg(ConstRequestPtr &_msg) {
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");

  if(_msg->request() == "object_list") {
    msgs::String_V src;
    response.set_type(src.GetTypeName());
    fill_list_msg(src);
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    this->srguiPub->Publish(response);
  }
  else if(_msg->request() == "object_data") {
    response.set_response("success");
    msgs::SceneObject src;
    response.set_type(src.GetTypeName());
    if(_msg->has_data() && fill_object_msg(_msg->data(), src)) {
      msgs::Request request;
      request.set_request(_msg->request());
      request.set_id(_msg->id());
      request.set_data(src.objectid());
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);

      this->framePub->Publish(request);
      this->srguiPub->Publish(response);
    }
    else {
      std::ostringstream tmp;
      tmp << "failure: " << _msg->data() << " not an object known to the objectinstantiator";
      response.set_response(tmp.str());
      this->srguiPub->Publish(response);
    }
  }
  else if(_msg->request() == "object_repository") {
    msgs::String_V src;
    response.set_type(src.GetTypeName());
    fill_repository_msg(src);
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    this->srguiPub->Publish(response);
  }
}

void ObjectInstantiatorPlugin::OnUpdate() {
  common::Time now = common::Time::GetWallTime();

  if(now >= this->next_spawn) {
    common::Time old_spawn = this->next_spawn;
    this->next_spawn = now + this->object_lifetime + this->object_lifetime + this->object_lifetime;
    std::map< std::string, SceneObject>::iterator it;
    for(it = object_spawn_list.begin(); it != object_spawn_list.end(); it++) {
      if(it->second.spawntime < now) {
        sdf::SDF _sdf;
        if(it->second.expiretime <= this->next_expire)
          this->next_expire = it->second.expiretime;
        _sdf.SetFromString(it->second.sdf_data);
        this->world->InsertModel(_sdf);
        it->second.model = this->world->GetModel(it->first);
        object_list[it->first] = it->second;
        object_spawn_list.erase(it);
      }
      else if(it->second.spawntime < this->next_spawn) {
        this->next_spawn = it->second.spawntime;
      }
    }
  }

  if(now >= this->next_expire) {
    common::Time old_expire = this->next_expire;
    this->next_expire = now + this->object_lifetime + this->object_lifetime + this->object_lifetime;
    std::map< std::string, SceneObject>::iterator it;
    for(it = object_list.begin(); it != object_list.end(); it++) {
      if(it->second.expiretime < now) {
        it->second.model->GetParent()->RemoveChild(it->first);
        object_list.erase(it);
      }
      else if(it->second.expiretime < this->next_expire) {
        this->next_expire = it->second.expiretime;
      }
    }
  }
}

bool ObjectInstantiatorPlugin::fill_object_msg(std::string name, msgs::SceneObject &_msg) {
  std::map< std::string, SceneObject>::iterator it =  object_list.find(name);
  if(it != object_list.end()) {
    _msg.set_object_type(it->second.type);
    const math::Pose pose = it->second.model->GetWorldPose();
    _msg.set_pos_x(pose.pos.x);
    _msg.set_pos_y(pose.pos.y);
    _msg.set_pos_z(pose.pos.z);
    _msg.set_ori_w(pose.rot.w);
    _msg.set_ori_x(pose.rot.x);
    _msg.set_ori_y(pose.rot.y);
    _msg.set_ori_z(pose.rot.z);
    _msg.set_frame(it->second.frame);
    _msg.set_child_frame(it->second.child_frame);
    _msg.set_objectid(it->second.objectid);
    _msg.set_name(it->first);
    return true;
  }
  
  return false;
}

void ObjectInstantiatorPlugin::fill_list_msg(msgs::String_V &_msg) {
  std::map<std::string, SceneObject>::iterator it;
  for(it = object_list.begin(); it != object_list.end(); it++) {
    _msg.add_data(it->first);
  }
}

void ObjectInstantiatorPlugin::fill_repository_msg(msgs::String_V &_msg) {
  std::map<std::string, std::string>::iterator it;
  for(it = objects.begin(); it != objects.end(); it++) {
    _msg.add_data(it->first);
  }
}

void ObjectInstantiatorPlugin::OnSceneObjectMsg(ConstMessage_VPtr &_msg) {
  // add new models
  msgs::SceneObject obj;
  if(_msg->msgtype() != obj.GetTypeName())
    return;

  int n_msgs = _msg->msgsdata_size();
  for(int n=0; n<n_msgs; n++) {
    obj.ParseFromString(_msg->msgsdata(n));
    sdf::SDF _sdf;
    std::string sdf_data;
    sdf_data = objects[obj.object_type()];
    double oriw = 0.0;
    double orix = 0.0;
    double oriy = 0.0;
    double oriz = 0.0;
    std::string name = "spawned_object";
    if(obj.has_ori_w())
      oriw = obj.ori_w();
    if(obj.has_ori_x())
      orix = obj.ori_x();
    if(obj.has_ori_y())
      oriy = obj.ori_y();
    if(obj.has_ori_z())
      oriz = obj.ori_z();
    if(obj.has_name())
      name = obj.name();

    std::string modelname = set_sdf_values(sdf_data, name, obj.pos_x(), obj.pos_y(), obj.pos_z(), oriw, orix, oriy, oriz);
    SceneObject so;
    so.type = obj.object_type();
    so.frame = obj.frame();
    so.child_frame = obj.child_frame();
    so.objectid = obj.objectid();

    if(obj.has_spawntime()) {
      so.spawntime = common::Time(obj.spawntime());
    }
    else {
      so.spawntime = common::Time::GetWallTime();
    }
    so.expiretime = so.spawntime + object_lifetime;
    if(so.spawntime <= this->next_spawn)
      this->next_spawn = so.spawntime;
    if(so.spawntime <= common::Time::GetWallTime()) {
      if(so.expiretime <= this->next_expire)
        this->next_expire = so.expiretime;
      _sdf.SetFromString(sdf_data);
      this->world->InsertModel(_sdf);
      so.model = this->world->GetModel(modelname);
      object_list[modelname] = so;
    }
    else {
      so.sdf_data = sdf_data;
      object_spawn_list[modelname] = so;
    }
  }
}

std::string ObjectInstantiatorPlugin::set_sdf_values(std::string &_sdf, std::string name, double pos_x, double pos_y, double pos_z, double ori_w, double ori_x, double ori_y, double ori_z) {
  std::ostringstream converter;
  std::string modelname;
  converter << name << "_" << this->object_count;
  modelname = converter.str();
  object_count++;
  sdf_replace(_sdf, "@NAME@", modelname);

  converter.str("");
  converter << pos_x;
  sdf_replace(_sdf, "@POSX@", converter.str());

  converter.str("");
  converter << pos_y;
  sdf_replace(_sdf, "@POSY@", converter.str());

  converter.str("");
  converter << pos_z;
  sdf_replace(_sdf, "@POSZ@", converter.str());

  converter.str("");
  converter << ori_w;
  sdf_replace(_sdf, "@ORIW@", converter.str());

  converter.str("");
  converter << ori_x;
  sdf_replace(_sdf, "@ORIX@", converter.str());

  converter.str("");
  converter << ori_y;
  sdf_replace(_sdf, "@ORIY@", converter.str());

  converter.str("");
  converter << ori_z;
  sdf_replace(_sdf, "@ORIZ@", converter.str());

  return modelname;
}

void ObjectInstantiatorPlugin::sdf_replace(std::string &text, std::string from, std::string to) {
  size_t position = text.find(from);
  size_t length = from.length();
  while(position != std::string::npos) {
    text.erase(position, length);
    text.insert(position, to);
    position = text.find(from);
  }
}
