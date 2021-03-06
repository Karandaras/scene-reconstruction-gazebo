/*
 * Copyright 2012 Nate Koenig
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
/* Desc: The base joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _JOINT_HH_
#define _JOINT_HH_

#include <string>

#include <boost/any.hpp>

#include "gazebo/common/Event.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/physics/JointState.hh"
#include "gazebo/physics/Base.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Joint Joint.hh physics/physics.hh
    /// \brief Base class for all joints
    class Joint : public Base
    {
      /// \enum Attribute
      /// \brief Joint attribute types.
      public: enum Attribute
              {
                /// \brief Fudge factor.
                FUDGE_FACTOR,

                /// \brief Suspension error reduction parameter.
                SUSPENSION_ERP,

                /// \brief Suspension constraint force mixing.
                SUSPENSION_CFM,

                /// \brief Stop limit error reduction parameter.
                STOP_ERP,

                /// \brief Stop limit constraint force mixing.
                STOP_CFM,

                /// \brief Error reduction parameter.
                ERP,

                /// \brief Constraint force mixing.
                CFM,

                /// \brief Maximum force.
                FMAX,

                /// \brief Velocity.
                VEL,

                /// \brief High stop angle.
                HI_STOP,

                /// \brief Low stop angle.
                LO_STOP
              };

      /// \brief Constructor
      /// \param[in] Joint parent
      public: explicit Joint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~Joint();

      /// \brief Set pose, parent and child links of a physics::Joint
      /// \param[in] _parent Parent link.
      /// \param[in] _child Child link.
      /// \param[in] _pose Pose of the link.
      public: void Load(LinkPtr _parent, LinkPtr _child,
                        const math::Pose &_pose);

      /// \brief Load physics::Joint from a SDF sdf::Element.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize a joint.
      public: virtual void Init();

      /// \brief Update the joint.
      public: void Update();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Reset the joint.
      public: virtual void Reset();

      /// \brief Set the joint state.
      /// \param[in] _state Joint state
      public: void SetState(const JointState &_state);

      /// \brief Set the model this joint belongs too.
      /// \param[in] _model Pointer to a model.
      public: void SetModel(ModelPtr _model);

      /// \brief Get the link to which the joint is attached according
      /// the _index.
      /// \param[in] _index Index of the link to retreive.
      /// \return Pointer to the request link. NULL if the index was
      /// invalid.
      public: virtual LinkPtr GetJointLink(int _index) const = 0;

      /// \brief Determines of the two bodies are connected by a joint.
      /// \param[in] _one First link.
      /// \param[in] _two Second link.
      /// \return True if the two links are connected by a joint.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const = 0;

      /// \brief Attach the two bodies with this joint.
      /// \param[in] _parent Parent link.
      /// \param[in] _child Child link.
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);

      /// \brief Detach this joint from all links.
      public: virtual void Detach();

      /// \brief Set the axis of rotation.
      /// \param[in] _index Index of the axis to set.
      /// \param[in] _axis Axis value.
      public: virtual void SetAxis(int _index, const math::Vector3 &_axis) = 0;

      /// \brief Set the joint damping.
      /// \param[in] _index Index of the axis to set.
      /// \param[in] _damping Damping value for the axis.
      public: virtual void SetDamping(int _index, double _damping) = 0;

      /// \brief Connect a boost::slot the the joint update signal.
      /// \param[in] _subscriber Callback for the connection.
      /// \return Connection pointer, which must be kept in scope.
      public: template<typename T>
              event::ConnectionPtr ConnectJointUpdate(T _subscriber)
              {return jointUpdate.Connect(_subscriber);}

      /// \brief Disconnect a boost::slot the the joint update signal.
      /// \param[in] _conn Connection to disconnect.
      public: void DisconnectJointUpdate(event::ConnectionPtr &_conn)
              {jointUpdate.Disconnect(_conn);}

      /// \brief Get the axis of rotation.
      /// \param[in] _index Index of the axis to get.
      /// \return Axis value for the provided index.
      public: math::Vector3 GetLocalAxis(int _index) const;

      /// \brief Get the axis of rotation in global cooridnate frame.
      /// \param[in] _index Index of the axis to get.
      /// \return Axis value for the provided index.
      public: virtual math::Vector3 GetGlobalAxis(int _index) const = 0;

      /// \brief Set the anchor point.
      /// \param[in] _index Indx of the axis.
      /// \param[in] _anchor Anchor value.
      public: virtual void SetAnchor(int _index,
                                     const math::Vector3 &_anchor) = 0;

      /// \brief Get the anchor point.
      /// \param[in] _index Index of the axis.
      /// \return Anchor value for the axis.
      public: virtual math::Vector3 GetAnchor(int _index) const = 0;

      /// \brief Set the high stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle High stop angle.
      public: virtual void SetHighStop(int _index,
                                       const math::Angle &_angle) = 0;

      /// \brief Set the low stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle Low stop angle.
      public: virtual void SetLowStop(int _index,
                                      const math::Angle &_angle) = 0;

      /// \brief Get the high stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \return Angle of the high stop value.
      public: virtual math::Angle GetHighStop(int _index) = 0;

      /// \brief Get the low stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \return Angle of the low stop value.
      public: virtual math::Angle GetLowStop(int _index) = 0;

      /// \brief Set the velocity of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _vel Velocity.
      public: virtual void SetVelocity(int _index, double _vel) = 0;

      /// \brief Get the rotation rate of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return The rotaional velocity of the joint axis.
      public: virtual double GetVelocity(int _index) const = 0;

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      public: virtual void SetForce(int _index, double _force);

      /// \brief @todo: not yet implemented.
      /// Get the internal forces at a this Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      /// \param[in] _index Index of the axis.
      /// \return The force applied to an axis.
      public: virtual double GetForce(int _index);

      /// \brief Set the max allowed force of an axis(index).
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Maximum force that can be applied to the axis.
      public: virtual void SetMaxForce(int _index, double _force) = 0;

      /// \brief Get the max allowed force of an axis(index).
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      /// \param[in] _index Index of the axis.
      /// \return The maximum force.
      public: virtual double GetMaxForce(int _index) = 0;

      /// \brief Get the angle of rotation of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the axis.
      public: math::Angle GetAngle(int _index) const;

      /// \brief Get the angle count.
      /// \return The number of DOF for the joint.
      public: virtual unsigned int GetAngleCount() const = 0;

      /// \brief If the Joint is static, Gazebo stores the state of
      /// this Joint as a scalar inside the Joint class, so
      /// this call will NOT move the joint dynamically for a static Model.
      /// But if this Model is not static, then it is updated dynamically,
      /// all the conencted children Link's are moved as a result of the
      /// Joint angle setting.  Dynamic Joint angle update is accomplished
      /// by calling JointController::SetJointPosition.
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle Angle to set the joint to.
      public: void SetAngle(int _index, math::Angle _angle);

      /// \brief Get the forces applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      /// \param[in] index The index of the link(0 or 1).
      /// \return Force applied to the link.
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const = 0;

      /// \brief Get the torque applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// Note that the unit of torque should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons-Meters.  If using
      /// imperial units (sorry), then unit of force is lb-force-inches
      /// not (lb-mass-inches), etc.
      /// \param[in] index The index of the link(0 or 1)
      /// \return Torque applied to the link.
      public: virtual math::Vector3 GetLinkTorque(
                  unsigned int _index) const = 0;

      /// \brief Set a parameter for the joint.
      /// \param[in] _attr Attribute to set.
      /// \param[in] _index Index of the axis.
      /// \param[in] _value Value of the attribute.
      public: virtual void SetAttribute(Attribute _attr, int _index,
                                        double _value) GAZEBO_DEPRECATED = 0;

      /// \brief Set a non-generic parameter for the joint.
      /// replaces SetAttribute(Attribute, int, double)
      /// \param[in] _key String key.
      /// \param[in] _index Index of the axis.
      /// \param[in] _value Value of the attribute.
      public: virtual void SetAttribute(const std::string &_key, int _index,
                                        const boost::any &_value) = 0;

      /// \brief Get the child link
      /// \return Pointer to the child link.
      public: LinkPtr GetChild() const;

      /// \brief Get the parent link.
      /// \return Pointer to the parent link.
      public: LinkPtr GetParent() const;

      /// \brief DEPRECATED
      /// \param[out] _msg Message to fill with joint's properties
      /// \sa Joint::FillMsg
      public: void FillJointMsg(msgs::Joint &_msg) GAZEBO_DEPRECATED;

      /// \brief Fill a joint message.
      /// \param[out] _msg Message to fill with this joint's properties.
      public: void FillMsg(msgs::Joint &_msg);

      /// \brief Get the angle of an axis helper function.
      /// \param[in] _index Index of the axis.
      /// \return Angle of the axis.
      protected: virtual math::Angle GetAngleImpl(int _index) const = 0;

      /// \brief Helper function to load a joint.
      /// \param[in] _pose Pose of the anchor.
      private: void LoadImpl(const math::Pose &_pose);

      /// \brief The first link this joint connects to
      protected: LinkPtr childLink;

      /// \brief The second link this joint connects to
      protected: LinkPtr parentLink;

      /// \brief Pointer to the parent model.
      protected: ModelPtr model;

      /// \brief Anchor pose.
      protected: math::Vector3 anchorPos;

      /// \brief Anchor link.
      protected: LinkPtr anchorLink;

      /// \brief Joint update event.
      private: event::EventT<void ()> jointUpdate;

      /// \brief joint damping_coefficient
      protected: double damping_coefficient;

      /// \brief Angle used when the joint is paret of a static model.
      private: math::Angle staticAngle;
    };
    /// \}
  }
}
#endif
