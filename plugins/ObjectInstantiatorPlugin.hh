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
          std::string         object;
          bool                visible;
          physics::ModelPtr   model;
          math::Pose          pose;
          common::Time        buffertime;
          std::string         query;
          std::string         frame;

          bool operator<(SceneObject comp) const {
            return buffertime<comp.buffertime;
          }
          bool operator==(std::string comp) const {
            return object==comp;
          }
      };

      std::map<std::string, SceneObject>  object_list;
      std::list<SceneObject>              object_buffer;
      transport::NodePtr                  node;
      transport::SubscriberPtr            objectSub,
                                          requestSub,
                                          statusSub;
      transport::PublisherPtr             srguiPub,
                                          framePub,
                                          objectPub,
                                          bufferPub,
                                          statusPub;
      unsigned int                        object_count;
      common::Time                        next_buffer;
      physics::WorldPtr                   world;
      boost::mutex                       *receiveMutex;
      std::list<msgs::SceneObject_V>      objectMsgs;
      std::vector<event::ConnectionPtr>   connections;
      math::Pose                          out_of_sight;
      math::Vector3                       position_offset;
      bool                                update_object_buffer;

    public: 
      void virtual Init();
      void virtual Reset();
      void virtual Load(physics::WorldPtr, sdf::ElementPtr);

    private:
      void OnSceneObjectMsg(ConstSceneObject_VPtr &_msg);
      void OnRequestMsg(ConstRequestPtr &_msg);
      void OnStatusMsg(ConstRequestPtr &_msg);
      void OnUpdate();
      void ProcessSceneObjectMsgs();
      void UpdateObjects(common::Time now);
      void update_object(SceneObject obj);
      bool fill_object_msg(std::string name, msgs::SceneObject &_msg);
      void fill_list_msg(msgs::GzString_V &_msg);
      void fill_repository_msg(msgs::GzString_V &_msg);
      void fill_buffer_msg(msgs::Message_V &_msg);
  };
  GZ_REGISTER_WORLD_PLUGIN(ObjectInstantiatorPlugin)
} 
#endif
