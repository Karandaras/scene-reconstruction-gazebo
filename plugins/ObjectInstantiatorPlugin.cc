#include <iostream>;

#include "common/SystemPaths.hh"

#include "ObjectInstantiatorPlugin.hh"

using namespace gazebo;

ObjectInstantiator::ObjectInstantiator() : WorldPlugin() 
{
  this->object_lifetime = common::Time(0.5);
}

void ObjectInstantiator::Init() {
  this->next_expire = common::Time::GetWallTime();
}

void ObjectInstantiator::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->object_count = 0;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

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
        if(!objectElem->GetElement("name")->GetValue()->Get(gname)) {
      	  gzerr << "<name> is not a string, leaving out this <object>\n";
        }
        else {
          if(objectElem->HasElement("filename")) {
            std::string filename;
            if(!objectElem->GetElement("filename")->GetValue()->Get(filename)) {
          	  gzerr << "<filename> is not a string, leaving out this <object>\n";
            }
            
            std::string _filename = common::SystemPaths::Instance()->FindFileWithGazeboPaths(filename);
            std::ifstream ifile(_filename);

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

void ObjectInstantiator::OnRequestMsg(ConstRequestPtr &_msg) {
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
      fill_object_msg(src);
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

void ObjectInstantiator::OnUpdate() {
  common::Time now = common::Time::GetWallTime();

  if(now >= this->next_spawn) {
    common::Time old_spawn = this->next_spawn;
    this->next_spawn = now + this->object_lifetime + this->object_lifetime + this->object_lifetime;
    std::map< std::string, SceneObject>::iterator it;
    for(it = object_spawn_list.begin(); it != object_spawn_list.end(); it++) {
      if(it->second.spawntime < now) {
        if(it->second.expiretime <= this->next_expire)
          this->next_expire = it->second.expiretime;
        _sdf.SetFromString(it->second.sdf_data);
        this->world->InsertModel(_sdf);
        it->second.model = this->world->GetModel(modelname);
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
        this->world->rootElement->RemoveChild(it->second.model->GetID());
        object_list.erase(it);
      }
      else if(it->second.expiretime < this->next_expire) {
        this->next_expire = it->second.expiretime;
      }
    }
  }
}

bool ObjectInstantiator::fill_object_msg(std::string name, msgs::SceneObject &_msg) {
  std::map< std::string, SceneObject>::iterator it =  object_list.find(name);
  if(it != object_list.end()) {
    _msg.add_object_type(it->second.type);
    const math::Pose pose = it->second.model->GetWorldPose();
    _msg.add_pos_x(pose.pos.x);
    _msg.add_pos_y(pose.pos.y);
    _msg.add_pos_z(pose.pos.z);
    _msg.add_rot_w(pose.rot.w);
    _msg.add_rot_x(pose.rot.x);
    _msg.add_rot_y(pose.rot.y);
    _msg.add_rot_z(pose.rot.z);
    _msg.add_frame(it->second.frame);
    _msg.add_child_frame(it->second.child_frame);
    _msg.add_objectid(it->second.objectid);
    _msg.add_name(it->first);
    return true;
  }
  
  return false;
}

void ObjectInstantiator::fill_list_msg(msgs::String_V &_msg) {
  std::map< std::string, SceneObject>::iterator it;
  for(it = object_list.begin(); it != object_list.end(); it++) {
    _msg.add_name(it->first);
  }
}

void ObjectInstantiator::fill_repository_msg(msgs::String_V &_msg) {
  std::map< std::string, std::string>::iterator it;
  for(it = objects.begin(); it != objects.end(); it++) {
    _msg.add_data(it->first);
  }
}

void ObjectInstantiator::OnSceneObjectMsg(ConstMessage_VPtr &_msg) {
  // add new models
  msgs::SceneObject obj;
  if(_msg->type() != obj.GetTypeName())
    return;

  for(int n=0; n<_msg->msgdata(); n++) {
    obj.ParseFromString(_msg->msgdata(n));
    sdf::SDF _sdf;
    std::string sdf_data;
    sdf_data = objects[obj.object_type()];
    double oriw = 0.0;
    double orix = 0.0;
    double oriy = 0.0;
    double oriz = 0.0;
    std::string name = "spawned_object";
    if(obj.has_ori_w())
      oriw = obj.rot_w();
    if(obj.has_ori_x())
      orix = obj.rot_x();
    if(obj.has_ori_y())
      oriy = obj.rot_y();
    if(obj.has_ori_z())
      oriz = obj.rot_z();
    if(obj.has_name())
      name = obj.name();

    std::string modelname = set_sdf_values(sdf_data, name, obj.pos_x(), obj.pos_y(), obj.pos_z(), oriw, orix, oriy, oriz);
    SceneObject so;
    so.type = obj.object_type();
    so.frame = obj.frame(;
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

std::string ObjectInstantiator::set_sdf_values(std::string &_sdf, std::string name, double pos_x, double pos_y, double pos_z, double ori_w, double ori_x, double ori_y, double ori_z) {
  std::ostringstream converter;
  std::string modelname;
  converter << name << "_" << this->object_count;
  modelname = converter.str();
  object_count++;
  replace(_sdf, "@NAME@", modelname);

  converter.str("");
  converter << pos_x;
  replace(_sdf, "@POSX@", converter.str());

  converter.str("");
  converter << pos_y;
  replace(_sdf, "@POSY@", converter.str());

  converter.str("");
  converter << pos_z;
  replace(_sdf, "@POSZ@", converter.str());

  converter.str("");
  converter << ori_w;
  replace(_sdf, "@ORIW@", converter.str());

  converter.str("");
  converter << ori_x;
  replace(_sdf, "@ORIX@", converter.str());

  converter.str("");
  converter << ori_y;
  replace(_sdf, "@ORIY@", converter.str());

  converter.str("");
  converter << ori_z;
  replace(_sdf, "@ORIZ@", converter.str());

  return modelname;
}

void replace(std::string &text, std::string search, std::string replace) {
  size_t position = text.find(search);
  size_t length = search.length();
  while(position != std::string::npos) {
    text.erase(position, length);
    text.insert(position, replace);
    position = text.find(search);
  }
}
