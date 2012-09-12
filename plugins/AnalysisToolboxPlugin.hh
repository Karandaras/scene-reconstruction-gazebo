#ifndef __GAZEBO_ANALYSIS_PLUGIN_HH__
#define __GAZEBO_ANALYSIS_PLUGIN_HH__

#include <vector>

#include "common/common.h"
#include "physics/physics.h"
#include "rendering/RenderTypes.hh"
#include "gazebo.hh"

namespace gazebo
{
  class AnalysisToolboxPlugin : public WorldPlugin
  {
    public:
      AnalysisToolboxPlugin();

    private:
      physics::WorldPtr                   world;
      std::vector<event::ConnectionPtr>   connections;
      rendering::ScenePtr                 scene;

      transport::SubscriberPtr            statusSub;
      transport::PublisherPtr             statusPub,
                                          requestPub;

    public: 
      void virtual Init();
      void virtual Reset();
      void virtual Load(physics::WorldPtr, sdf::ElementPtr);

    private:
      void OnUpdate();
  };
  GZ_REGISTER_WORLD_PLUGIN(AnalysisToolboxPlugin)
} 
#endif
