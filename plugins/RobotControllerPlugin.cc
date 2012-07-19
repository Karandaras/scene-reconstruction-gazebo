/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "physics/physics.h"
#include "transport/transport.h"
#include "plugins/RobotControllerPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(RobotControllerPlugin)

/////////////////////////////////////////////////
RobotControllerPlugin::RobotControllerPlugin()
{
}

/////////////////////////////////////////////////
void RobotControllerPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  if (!_sdf->HasElement("left_joint"))
    gzerr << "DiffDrive plugin missing <left_joint> element\n";

  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->GetValueString());

  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->GetValueString() << "]\n";

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&RobotControllerPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void RobotControllerPlugin::Init()
{
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnVelMsg(ConstPosePtr &_msg)
{
}

/////////////////////////////////////////////////
void RobotControllerPlugin::OnUpdate()
{
}
