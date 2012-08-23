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
          std::string sdf_data;
          std::string frame;
          std::string child_frame;
          std::string objectid;
          common::Time spawntime;
          common::Time expiretime;
      } SceneObject;

      std::map< std::string, SceneObject> object_list;
      std::map< std::string, SceneObject> object_spawn_list;
      std::map< std::string, std::string> objects;
      transport::NodePtr                  node;
      transport::SubscriberPtr            objectSub;
      transport::PublisherPtr             srguiPub,
                                          framePub;
      unsigned int                        object_count;
      common::Time                        object_lifetime,
                                          next_spawn,
                                          next_expire;

    public: 
      void virtual Init();
      void virtual Load(physics::WorldPtr, sdf::ElementPtr);

    private:
      void OnSceneObjectMsg(ConstSceneObject_VPtr &_msg);
      void OnRequestMsg(ConstRequestPtr &_msg);
      void OnUpdate();
      void fill_object_v_msg(msgs::SceneObject_V &_msg);
      void fill_repository_msg(msgs::String_V &_msg);
      void process_objects_overlapping(ConstSceneObject_VPtr &_msg);
      void process_objects_non_overlapping(ConstSceneObject_VPtr &_msg);
      void process_objects_temporary(ConstSceneObject_VPtr &_msg);
      std::string set_sdf_values(std::string &_sdf, std::string name, double pos_x, double pos_y, double pos_z, double rot_w, double rot_x, double rot_y, double rot_z);
      void replace(std::string &text, std::string search, std::string replace);
  };
  GZ_REGISTER_WORLD_PLUGIN(ObjectInstantiator)
} 
