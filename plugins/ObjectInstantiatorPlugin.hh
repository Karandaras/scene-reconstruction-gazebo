#ifndef __GAZEBO_OBJECTINSTANTIATOR_PLUGIN_HH__
#define __GAZEBO_OBJECTINSTANTIATOR_PLUGIN_HH__

#include <boost/thread/mutex.hpp>

#include "gazebo.hh"
#include "common/common.h"
#include "transport/TransportTypes.hh"
#include "physics/PhysicsTypes.hh"

namespace gazebo
{
/** @class ObjectInstantiatorPlugin "objectinstantiatorplugin.h"
 *  ObjectInstantiator Plugin for Scene Reconstruction using the Gazebo simulator
 *  @author Bastian Klingen
 */
  class ObjectInstantiatorPlugin : public WorldPlugin
  {
    public:
      /** Constructor */
      ObjectInstantiatorPlugin();

    private:
      /** struct for objects inside the scene */
      struct SceneObject{
          /** name of the model inside the simulator */
          std::string         object;
          /** visibility of the object */
          bool                visible;
          /** pointer to the model of the object */
          physics::ModelPtr   model;
          /** pose of the object */
          math::Pose          pose;
          /** time when this object has to be used */
          common::Time        buffertime;
          /** query for all documents that may be related to this object */
          std::string         query;
          /** frame this object was captured with */
          std::string         frame;

          /** comparison operator to use find */
          bool operator<(SceneObject comp) const {
            return buffertime<comp.buffertime;
          }
          /** equality operator */
          bool operator==(std::string comp) const {
            return object==comp;
          }
      };

      std::map<std::string, SceneObject>  object_list;
      std::map<std::string, std::string>  object_names;
      std::list<SceneObject>              object_buffer;
      transport::NodePtr                  node;
      transport::SubscriberPtr            objectSub,
                                          requestSub,
                                          bufferSub;
      transport::PublisherPtr             srguiPub,
                                          framePub,
                                          requestPub,
                                          objectPub,
//                                          bufferPub,
                                          drawingPub,
                                          statusPub;
      common::Time                        next_buffer;
      physics::WorldPtr                   world;
      boost::mutex                       *receiveMutex;
      std::list<msgs::Message_V>          objectMsgs;
      std::vector<event::ConnectionPtr>   connections;
      math::Pose                          out_of_sight;
      math::Vector3                       position_offset;
      bool                                __available;

    public: 
      /** initialisation of the plugin */
      void virtual Init();
      /** reset of the plugin */
      void virtual Reset();
      /** loading routine of the plugin */
      void virtual Load(physics::WorldPtr, sdf::ElementPtr);

    private:
      /** Callback for received Message_V messages to enqueue them for buffering
        * @param _msg Message_V message containing data for buffering
        */
      void OnSceneObjectMsg(ConstMessage_VPtr &_msg);
      /** Callback for received SceneObject messages to visualize buffer content
        * @param _msg SceneObject message containing needed data for visualization
        */
      void OnObjectMsg(ConstSceneObjectPtr &_msg);
      /** Callback for received Request messages
        * @param _msg Request message
        */
      void OnRequestMsg(ConstRequestPtr &_msg);
      /** Update procedure connected to Gazebo's update
        */
      void OnUpdate();
      /** Processes the message queue and builds the buffer
        */
      void ProcessSceneObjectMsgs();
      /** Updates the Objects using the buffer
        * @param now The current time for reference with the buffer timestamps
        */
      void UpdateObjects(common::Time now);
      /** Updates a single object, called by UpdateObjects
        * @param obj the buffered object used for updating
        */
      void update_object(SceneObject obj);
      /** Fills a SceneObject message for an object specified by its name
        * @param name Name of the Object
        * @param _msg Message that gets filled
        */
      bool fill_object_msg(std::string name, msgs::SceneObject &_msg);
      /** Fills a message with the names of the affected objects
        * @param _msg Message that gets filled
        */
      void fill_list_msg(msgs::GzString_V &_msg);
      /** Fills a message with the current buffer content
        * @param _msg Message that gets filled
        */
//      void fill_buffer_msg(msgs::Message_V &_msg);
  };
  GZ_REGISTER_WORLD_PLUGIN(ObjectInstantiatorPlugin)
} 
#endif
