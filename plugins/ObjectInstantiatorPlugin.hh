#ifndef __GAZEBO_OBJECTINSTANTIATOR_PLUGIN_HH__
#define __GAZEBO_OBJECTINSTANTIATOR_PLUGIN_HH__

#include "gazebo.hh"
#include "/common/common.h"

namespace gazebo
{
  class ObjectInstantiator : public WorldPlugin
  {
    public:
      ObjectInstantiator();

    private:
      typedef struct {
          std::string type;
          ModelPtr    model;
          std::string frame;
          std::string child_frame;
          std::string objectid;
      } SceneObject;

      std::map< std::string, SceneObject> object_list;
      std::map< std::string, std::string> objects;
      transport::NodePtr node;
      transport::SubscriberPtr objectSub;
      transport::PublisherPtr srguiPub, framePub;

    public: 
      void Load(physics::WorldPtr, sdf::ElementPtr);

    private:
      void OnSceneObjectMsg(ConstSceneObjectPtr &_msg);
      void OnRequestMsg(ConstRequestPtr &_msg);
      void fill_object_v_msg(msgs::SceneObject_V &_msg);
      void fill_repository_msg(msgs::String_V &_msg);

  };
  GZ_REGISTER_WORLD_PLUGIN(ObjectInstantiator)
} 
