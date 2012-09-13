#ifndef __GAZEBO_OBJECTINSTANTIATOR_PLUGIN_HH__
#define __GAZEBO_OBJECTINSTANTIATOR_PLUGIN_HH__

#include <boost/thread/mutex.hpp>

#include "gazebo.hh"
#include "common/common.h"
#include "transport/TransportTypes.hh"
#include "physics/PhysicsTypes.hh"

namespace gazebo
{
  class ObjectInstantiatorPlugin : public WorldPlugin
  {
    public:
      ObjectInstantiatorPlugin();

    private:
      struct SceneObject{
          std::string         name;
          std::string         type;
          physics::ModelPtr   model;
          std::string         sdf_data;
          std::string         frame;
          std::string         child_frame;
          std::string         objectids;
          common::Time        spawntime;
          common::Time        expiretime;

          bool operator<(SceneObject comp) const {
            return spawntime<comp.spawntime;
          }
          bool operator==(std::string comp) const {
            return name==comp;
          }
      };

      std::list< SceneObject >            object_list;
      std::list< SceneObject >            object_spawn_list;
      std::map<std::string, std::string>  objects;
      transport::NodePtr                  node;
      transport::SubscriberPtr            objectSub,
                                          requestSub,
                                          statusSub;
      transport::PublisherPtr             srguiPub,
                                          framePub,
                                          statusPub;
      unsigned int                        object_count;
      common::Time                        object_lifetime,
                                          next_spawn,
                                          next_expire;
      physics::WorldPtr                   world;
      boost::mutex                       *receiveMutex;
      std::list<msgs::Message_V>          objectMsgs;
      std::vector<event::ConnectionPtr>   connections;

    public: 
      void virtual Init();
      void virtual Reset();
      void virtual Load(physics::WorldPtr, sdf::ElementPtr);

    private:
      void OnSceneObjectMsg(ConstMessage_VPtr &_msg);
      void OnRequestMsg(ConstRequestPtr &_msg);
      void OnStatusMsg(ConstRequestPtr &_msg);
      void OnUpdate();
      void ProcessSceneObjectMsgs();
      void SpawnObjects(common::Time now);
      void DeleteObjects(common::Time now);
      bool fill_object_msg(std::string name, msgs::SceneObject &_msg);
      void fill_list_msg(msgs::String_V &_msg);
      void fill_repository_msg(msgs::String_V &_msg);
      std::string set_sdf_values(std::string &_sdf, std::string name, double pos_x, double pos_y, double pos_z, double rot_r, double rot_p, double rot_y);
      void sdf_replace(std::string &text, std::string from, std::string to);
  };
  GZ_REGISTER_WORLD_PLUGIN(ObjectInstantiatorPlugin)
} 
#endif
