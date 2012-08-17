#include <iostream>;

#include "common/SystemPaths.hh"

#include "ObjectInstantiatorPlugin.hh"

using namespace gazebo;

ObjectInstantiator::ObjectInstantiator() : WorldPlugin() 
{
  this->object_lifetime = common::Time(10.0);
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
    if(settingsElem->HasElement("overlapping")) {
      if(!settingsElem->GetElement("overlapping")->GetValue()->Get(overlapping)) {
    	  gzerr << "<overlapping> is not a bool, defaulting to true\n";
        overlapping = true;
      }
    }
    else {
      overlapping = true;
    }

    if(settingsElem->HasElement("temporary")) {
      if(!settingsElem->GetElement("temporary")->GetValue()->Get(overlapping)) {
    	  gzerr << "<temporary> is not a bool, defaulting to false\n";
        temporary = false;
      }
    }
    else {
      temporary = false;
    }

    if(settingsElem->HasElement("object_lifetime")) {
      double lifetime;
      if(!settingsElem->GetElement("object_lifetime")->GetValue()->Get(lifetime)) {
    	  gzerr << "<object_lifetime> is not a double, defaulting to 10 seconds\n";
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

void ObjectInstantiator::OnSceneObjectMsg(ConstSceneObject_VPtr &_msg) {
  if(overlapping)
    process_objects_overlapping(_msg);
  else if(temporary)
    process_objects_temporary(_msg);
  else
    process_objects_non_overlapping(_msg);
}

void ObjectInstantiator::OnRequestMsg(ConstRequestPtr &_msg) {
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");

  if(_msg->request() == "object_list") {
    msgs::SceneObject_V src;
    response.set_type(src.GetTypeName());
    fill_object_v_msg(src);
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
  if (this->temporary)
    return;

  if(common::Time::GetWallTime() < this->next_expire)
    return;
  
  common::Time now = common::Time::GetWallTime();
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

void ObjectInstantiator::fill_object_v_msg(msgs::SceneObject_V &_msg) {
  std::map< std::string, SceneObject>::iterator it;
  for(it = object_list.begin(); it != object_list.end(); it++) {
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
  }
}

void ObjectInstantiator::fill_repository_msg(msgs::String_V &_msg) {
  std::map< std::string, std::string>::iterator it;
  for(it = objects.begin(); it != objects.end(); it++) {
    _msg.add_data(it->first);
    _msg.add_data(it->second);
  }
}

void ObjectInstantiator::process_objects_overlapping(ConstSceneObject_VPtr &_msg) {
  // add new models
  for(int n=0; n<_msg->object_type_size(); n++) {
    sdf::SDF _sdf;
    std::string sdf_data;
    sdf_data = objects[_msg->object_type(n)];
    std::string modelname = set_sdf_values(sdf_data, _msg->name(n), _msg->pos_x(n), _msg->pos_y(n), _msg->pos_z(n), _msg->rot_w(n), _msg->rot_x(n), _msg->rot_y(n), _msg->rot_z(n));
    SceneObject so;
    so.type = _msg->object_type(n);
    so.frame = _msg->frame(n);
    so.child_frame = _msg->child_frame(n);
    so.objectid = _msg->objectid(n);
    _sdf.SetFromString(sdf_data);
    this->world->InsertModel(_sdf);
    so.model = this->world->GetModel(modelname);
    if(so.expiretime <= this->next_expire)
      this->next_expire = so.expiretime;
    object_list[modelname] = so;
  }
}

void ObjectInstantiator::process_objects_non_overlapping(ConstSceneObject_VPtr &_msg) {
  // add new models
  for(int n=0; n<_msg->object_type_size(); n++) {
    // check for overlapping existing models
//    for(it = object_list.begin(); it != object_list.end(); it++) {
//    }

    sdf::SDF _sdf;
    std::string sdf_data;
    sdf_data = objects[_msg->object_type(n)];
    std::string modelname = set_sdf_values(sdf_data, _msg->name(n), _msg->pos_x(n), _msg->pos_y(n), _msg->pos_z(n), _msg->rot_w(n), _msg->rot_x(n), _msg->rot_y(n), _msg->rot_z(n));
    SceneObject so;
    so.type = _msg->object_type(n);
    so.frame = _msg->frame(n);
    so.child_frame = _msg->child_frame(n);
    so.objectid = _msg->objectid(n);
    _sdf.SetFromString(sdf_data);
    this->world->InsertModel(_sdf);
    so.model = this->world->GetModel(modelname);
    so.expiretime = common::Time::GetWallTime()+this->object_lifetime;
    if(so.expiretime <= this->next_expire)
      this->next_expire = so.expiretime;
    object_list[modelname] = so;
  }
}

void ObjectInstantiator::process_objects_temporary(ConstSceneObject_VPtr &_msg) {
  // remove all previouse models
  for(it = object_list.begin(); it != object_list.end(); it++) {
    this->world->rootElement->RemoveChild(it->second.model->GetID());
  }
  object_list.clear();

  // add new models
  for(int n=0; n<_msg->object_type_size(); n++) {
    sdf::SDF _sdf;
    std::string sdf_data;
    sdf_data = objects[_msg->object_type(n)];
    std::string modelname = set_sdf_values(sdf_data, _msg->name(n), _msg->pos_x(n), _msg->pos_y(n), _msg->pos_z(n), _msg->rot_w(n), _msg->rot_x(n), _msg->rot_y(n), _msg->rot_z(n));
    SceneObject so;
    so.type = _msg->object_type(n);
    so.frame = _msg->frame(n);
    so.child_frame = _msg->child_frame(n);
    so.objectid = _msg->objectid(n);
    _sdf.SetFromString(sdf_data);
    this->world->InsertModel(_sdf);
    so.model = this->world->GetModel(modelname);
    object_list[modelname] = so;
  }
}

std::string ObjectInstantiator::set_sdf_values(std::string &_sdf, std::string name, double pos_x, double pos_y, double pos_z, double rot_w, double rot_x, double rot_y, double rot_z) {
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
  converter << rot_w;
  replace(_sdf, "@ROTW@", converter.str());

  converter.str("");
  converter << rot_x;
  replace(_sdf, "@ROTX@", converter.str());

  converter.str("");
  converter << rot_y;
  replace(_sdf, "@ROTY@", converter.str());

  converter.str("");
  converter << rot_z;
  replace(_sdf, "@ROTZ@", converter.str());

  return modelname;
}

void replace(std::string &text, std::string search, std::string replace) {
  size_t position = text.find(search);
  size_t length = search.length();
  text.erase(position, length);
  text.insert(position, replace);
}
