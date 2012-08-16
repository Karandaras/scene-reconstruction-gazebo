#include <iostream>;

#include "common/SystemPaths.hh"

#include "ObjectInstantiatorPlugin.hh"

using namespace gazebo;

ObjectInstantiator::ObjectInstantiator() : WorldPlugin() 
{
}

void ObjectInstantiator::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  this->objectSub = this->node->Subscribe(std::string("~/SceneReconstruction/ObjectInstantiator/Object"), &ObjectInstantiatorPlugin::OnSceneObjectMsg, this);
  this->srguiPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/ObjectInstantiator/Response"));
  this->framePub = this->node->Advertise<msgs::Request>(std::string("~/SceneReconstruction/ObjectInstantiator/Request"));

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
    }
  }
}

void ObjectInstantiator::OnSceneObjectMsg(ConstSceneObjectPtr &_msg) {
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

