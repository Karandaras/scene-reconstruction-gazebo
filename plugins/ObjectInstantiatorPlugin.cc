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
  this->receiveMutex = new boost::mutex();
}

void ObjectInstantiatorPlugin::Init() {
  this->next_expire = common::Time(world->GetSimTime());
  this->next_spawn = common::Time(world->GetSimTime());
}

void ObjectInstantiatorPlugin::Reset() {
  this->next_expire = common::Time(world->GetSimTime());
  this->next_spawn = common::Time(world->GetSimTime());
  this->objectMsgs.clear();
  this->object_list.clear();
  this->object_spawn_list.clear();
  this->object_count = 0;
}

void ObjectInstantiatorPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->receiveMutex = new boost::mutex();
  this->object_count = 0;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  if(_sdf->HasElement("settings_lifetime")) {
    std::string tmplifetime;
    double lifetime;
    if(!_sdf->GetElement("settings_lifetime")->GetValue()->Get(tmplifetime)) {
  	  gzerr << "<settings_lifetime> is not readable, defaulting to 0.5 seconds\n";
    }
    else {
      char* t;
      lifetime = strtod(tmplifetime.c_str(), &t);
      if(*t != 0) {
        gzerr << "<settings_lifetime> not a double, defaulting to 0.5 seconds\n";
      }
      else {
        this->object_lifetime = lifetime;
      }
    }
  }

  sdf::ElementPtr objectElem;
  if (_sdf->HasElement("object")) {
    objectElem = _sdf->GetElement("object");
    std::string object;
    while(objectElem) {
      _sdf->GetElement("object")->GetValue()->Get(object);
      std::string name;
      std::string data;

      if(_sdf->HasElement("name_"+object)) {
        if(!_sdf->GetElement("name_"+object)->GetValue()->Get(name)) {
      	  gzerr << "<name_" << object << "> is not a string, leaving out this <object>\n";
        }
        else {
          if(_sdf->HasElement("filename_"+object)) {
            std::string sdf_filename;
            if(!_sdf->GetElement("filename_"+object)->GetValue()->Get(sdf_filename)) {
          	  gzerr << "<filename_" << object << "> is not a string, leaving out this <object>\n";
            }
            sdf_filename = common::SystemPaths::Instance()->GetModelPathExtension().substr(1) + "/" + sdf_filename;
            std::string _filename = common::SystemPaths::Instance()->FindFileWithGazeboPaths(sdf_filename);
            std::ifstream ifile(_filename.c_str());

            if(!ifile) {              
          	  gzerr << "\"" << sdf_filename << "\" does not exist, leaving out this <object>\n";              
            }
            else {
              std::ostringstream _data;
              std::string line;
              while(!ifile.eof()) {
                getline(ifile,line);
                _data << line << std::endl;
              }
              data = _data.str();

              std::string sdf_scale = "1 1 1";
              if(_sdf->HasElement("scale_"+object)) {
                if(!_sdf->GetElement("scale_"+object)->GetValue()->Get(sdf_scale)) {
              	  gzerr << "<scale_" << object << "> is not a string, leaving it out\n";
                }              
              }
              sdf_replace(data, std::string("@SCALE@"), sdf_scale);

              std::string sdf_height = "1";
              if(_sdf->HasElement("height_"+object)) {
                if(!_sdf->GetElement("height_"+object)->GetValue()->Get(sdf_height)) {
              	  gzerr << "<height_" << object << "> is not a string, leaving it out\n";
                }              
              }
              sdf_replace(data, std::string("@HEIGHT@"), sdf_height);

              std::string sdf_colorrgba = "0 0 0 1";
              if(_sdf->HasElement("colorrgba_"+object)) {
                if(!_sdf->GetElement("colorrgba_"+object)->GetValue()->Get(sdf_colorrgba)) {
              	  gzerr << "<colorrgba_" << object << "> is not a string, leaving it out\n";
                }
              }
              sdf_replace(data, std::string("@COLORRGBA@"), sdf_colorrgba);

              std::string sdf_colorscript = "Gazebo/FlatBlack";
              if(_sdf->HasElement("colorscript_"+object)) {
                if(!_sdf->GetElement("colorscript_"+object)->GetValue()->Get(sdf_colorscript)) {
              	  gzerr << "<colorscript_" << object << "> is not a string, leaving it out\n";
                }
              }
              sdf_replace(data, std::string("@COLORSCRIPT@"), sdf_colorscript);

              objects[name] = data;
              ifile.close();
            }
          }
          else {
        	  gzerr << "missing required element <filename_" << object << ">, leaving out this <object>\n";
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
  this->statusPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));

  msgs::Response response;
  response.set_id(-1);
  response.set_request("status");
  response.set_response("ObjectInstantiator");
  this->statusPub->Publish(response);

  this->objectSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Object"), &ObjectInstantiatorPlugin::OnSceneObjectMsg, this);
  this->requestSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Request"), &ObjectInstantiatorPlugin::OnRequestMsg, this);
  this->statusSub = this->node->Subscribe(std::string("~/SceneReconstruction/GUI/Availability/Request/ObjectInstantiator"), &ObjectInstantiatorPlugin::OnStatusMsg, this);
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
    msgs::SceneObject scob;
    response.set_type(scob.GetTypeName());
    if(_msg->has_data() && fill_object_msg(_msg->data(), scob)) {
      msgs::Request request;
      request.set_request(_msg->request());
      request.set_id(_msg->id());
      request.set_data(scob.objectids());
      std::string *serializedData = response.mutable_serialized_data();
      scob.SerializeToString(serializedData);

      this->framePub->Publish(request);
      this->srguiPub->Publish(response);
    }
    else {
      response.set_response("failure");
      msgs::String src;
      response.set_type(src.GetTypeName());
      src.set_data(_msg->data()+" not an object known to the objectinstantiator");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);

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
  else if(_msg->request() == "get_frame") {
    std::list<SceneObject>::iterator it =  find(object_list.begin(), object_list.end(), _msg->data());
    if(it != object_list.end()) {
      msgs::String src;
      response.set_type(src.GetTypeName());

      src.set_data(it->frame);

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
    msgs::String src;
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
  this->SpawnObjects(now);
  this->DeleteObjects(now);
}

void ObjectInstantiatorPlugin::SpawnObjects(common::Time now) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  if(now >= this->next_spawn) {
    this->next_spawn = now + this->object_lifetime + this->object_lifetime + this->object_lifetime;
    std::list<SceneObject>::iterator it;
    std::list<SceneObject> new_spawn_list;
    for(it = object_spawn_list.begin(); it != object_spawn_list.end(); it++) {
      if(it->spawntime < now && it->expiretime > now) {
        sdf::SDF _sdf;
        if(it->expiretime < this->next_expire)
          this->next_expire = it->expiretime;
        _sdf.SetFromString(it->sdf_data);
        this->world->InsertModel(_sdf);
        it->model = this->world->GetModel(it->name);
        object_list.push_back(*it);
      }
      else if(it->spawntime < this->next_spawn) {
        this->next_spawn = it->spawntime;
        new_spawn_list.push_back(*it);
      }
    }
    object_spawn_list = new_spawn_list;
    object_spawn_list.sort();
    object_list.sort();
  }
}

void ObjectInstantiatorPlugin::DeleteObjects(common::Time now) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  if(now >= this->next_expire) {
    this->next_expire = now + this->object_lifetime + this->object_lifetime + this->object_lifetime;
    std::list<SceneObject>::iterator it;
    std::list<SceneObject> new_object_list;
    for(it = object_list.begin(); it != object_list.end(); it++) {
      if(it->expiretime < now) {
        it->model->GetParent()->RemoveChild(it->name);
      }
      else if(it->expiretime < this->next_expire) {
        this->next_expire = it->expiretime;
        new_object_list.push_back(*it);
      }
    }
    object_list = new_object_list;
    object_list.sort();
  }
}

bool ObjectInstantiatorPlugin::fill_object_msg(std::string name, msgs::SceneObject &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::list<SceneObject>::iterator it =  find(object_list.begin(), object_list.end(), name);
  if(it != object_list.end()) {
    _msg.set_object_type(it->type);
    const math::Pose pose = it->model->GetWorldPose();
    _msg.set_pos_x(pose.pos.x);
    _msg.set_pos_y(pose.pos.y);
    _msg.set_pos_z(pose.pos.z);
    _msg.set_ori_w(pose.rot.w);
    _msg.set_ori_x(pose.rot.x);
    _msg.set_ori_y(pose.rot.y);
    _msg.set_ori_z(pose.rot.z);
    _msg.set_frame(it->frame);
    _msg.set_objectids(it->objectids);
    _msg.set_name(it->name);
    return true;
  }
  
  return false;
}

void ObjectInstantiatorPlugin::fill_list_msg(msgs::String_V &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::list<SceneObject>::iterator it;
  for(it = object_list.begin(); it != object_list.end(); it++) {
    _msg.add_data(it->name);
  }
}

void ObjectInstantiatorPlugin::fill_repository_msg(msgs::String_V &_msg) {
  std::map<std::string, std::string>::iterator it;
  for(it = objects.begin(); it != objects.end(); it++) {
    _msg.add_data(it->first);
  }
}

void ObjectInstantiatorPlugin::OnSceneObjectMsg(ConstMessage_VPtr &_msg) {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->objectMsgs.push_back(*_msg);
}

void ObjectInstantiatorPlugin::ProcessSceneObjectMsgs() {
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  std::list<msgs::Message_V>::iterator _msg;
  for (_msg = this->objectMsgs.begin(); _msg != this->objectMsgs.end(); ++_msg) {
    // add new models
    msgs::SceneObject obj;
    if(_msg->msgtype() == obj.GetTypeName()) {
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
        math::Quaternion q(oriw, orix, oriy, oriz);
        if(obj.has_name())
          name = obj.name();

        std::string modelname = set_sdf_values(sdf_data, name, obj.pos_x(), obj.pos_y(), obj.pos_z(), q.GetRoll(), q.GetPitch(), q.GetYaw());
        SceneObject so;
        so.type = obj.object_type();
        so.frame = obj.frame();
        so.objectids = obj.objectids();

        if(obj.has_spawntime()) {
          so.spawntime = common::Time(obj.spawntime());
        }
        else {
          so.spawntime = common::Time(world->GetSimTime());
        }
        so.expiretime = so.spawntime + object_lifetime;
        if(so.spawntime <= this->next_spawn)
          this->next_spawn = so.spawntime;
        if(so.spawntime <= world->GetSimTime()) {
          if(so.expiretime <= this->next_expire)
            this->next_expire = so.expiretime;
          _sdf.SetFromString(sdf_data);
          this->world->InsertModel(_sdf);
          so.model = this->world->GetModel(modelname);
          so.name = modelname;
          object_list.push_back(so);
        }
        else {
          so.sdf_data = sdf_data;
          so.name = modelname;
          object_spawn_list.push_back(so);
        }
      }
    }
  }

  object_list.sort();
  object_spawn_list.sort();
  objectMsgs.clear();
}

std::string ObjectInstantiatorPlugin::set_sdf_values(std::string &_sdf, std::string name, double pos_x, double pos_y, double pos_z, double rot_r, double rot_p, double rot_y) {
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
  converter << rot_r;
  sdf_replace(_sdf, "@ROTR@", converter.str());

  converter.str("");
  converter << rot_p;
  sdf_replace(_sdf, "@ROTP@", converter.str());

  converter.str("");
  converter << rot_y;
  sdf_replace(_sdf, "@ROTY@", converter.str());

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
