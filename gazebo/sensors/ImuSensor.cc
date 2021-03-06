/*
 * Copyright 2011 Nate Koenig
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
/* Desc: IMU sensor
 * Author: Matt Thompson
 * Date: 6 September 2008
*/


#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ImuSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("imu", ImuSensor)

//////////////////////////////////////////////////
ImuSensor::ImuSensor()
    : Sensor()
{
  std::cout << "NEW IMU\n";
}

//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  this->sdf->PrintValues("  ");

  if (this->sdf->HasElement("imu") &&
      this->sdf->GetElement("imu")->HasElement("topic") &&
      this->sdf->GetElement("imu")->GetValueString("topic")
      != "__default_topic__")
  {
    this->pub = this->node->Advertise<msgs::IMU>(
        this->sdf->GetElement("imu")->GetValueString("topic"));
  }
  else
  {
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName() + "/imu";
    boost::replace_all(topicName, "::", "/");

    this->pub = this->node->Advertise<msgs::IMU>(topicName);
  }
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->parentEntity = boost::shared_dynamic_cast<physics::Link>(
      this->world->GetEntity(this->parentName));

  if (!this->parentEntity)
  {
    gzthrow("IMU has invalid paret[" + this->parentName +
            "]. Must be a link\n");
  }
}

//////////////////////////////////////////////////
void ImuSensor::Init()
{
}

//////////////////////////////////////////////////
void ImuSensor::Fini()
{
}

//////////////////////////////////////////////////
math::Vector3 ImuSensor::GetAngularVelocity() const
{
  return msgs::Convert(this->imuMsg.angular_velocity());
}

//////////////////////////////////////////////////
math::Vector3 ImuSensor::GetLinearAcceleration() const
{
  return msgs::Convert(this->imuMsg.linear_acceleration());
}

//////////////////////////////////////////////////
void ImuSensor::UpdateImpl(bool /*_force*/)
{
  this->lastMeasurementTime = this->world->GetSimTime();

  this->imuMsg.set_entity_name(this->parentName);

  // Set the time stamp
  msgs::Set(this->imuMsg.mutable_stamp(), this->world->GetSimTime());

  // Set the IMU orientation
  msgs::Set(this->imuMsg.mutable_orientation(),
            this->parentEntity->GetWorldPose().rot);

  // Set the IMU angular velocity
  msgs::Set(this->imuMsg.mutable_angular_velocity(),
            this->parentEntity->GetWorldAngularVel());

  // Set the IMU linear acceleration
  msgs::Set(this->imuMsg.mutable_linear_acceleration(),
            this->parentEntity->GetWorldLinearAccel());

  if (this->pub)
    this->pub->Publish(this->imuMsg);
}
