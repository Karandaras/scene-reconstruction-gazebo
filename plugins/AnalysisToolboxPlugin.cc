#include "AnalysisToolboxPlugin.hh"

using namespace gazebo;

AnalysisToolboxPlugin::AnalysisToolboxPlugin() : WorldPlugin() 
{
}

void AnalysisToolboxPlugin::Init() {
}

void AnalysisToolboxPlugin::Reset() {
}

void AnalysisToolboxPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->scene = rendering::get_scene(this->world->GetName());

  this->statusSub = this->node->Subscribe(std::string("~/SceneReconstruction/GUI/Availability/Request/AnalysisToolbox"), &AnalysisToolboxPlugin::OnStatusMsg, this);
  this->statusPub = this->node->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));

  // connect update to worldupdate
  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
          boost::bind(&AnalysisToolboxPlugin::OnUpdate, this)));
}

void AnalysisToolboxPlugin::OnStatusMsg(ConstRequestPtr &_msg) {
  if(_msg->request() == "status") {
    msgs::Response response;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response("AnalysisToolbox");
    this->statusPub->Publish(response);
  }
}

void AnalysisToolboxPlugin::OnUpdate() {
  if(this->world->IsPaused())
    return;
}


/*  
    //make model transparent
    msgs::Request *requestMsg
    requestMsg = msgs::CreateRequest("set_transparency", this->modelName);
    requestMsg->set_dbl_data(0.75);
*/
