/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _sdcCar_hh_
#define _sdcCar_hh_

#include <string>
#include <vector>
#include <exception>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include "sdcIntersection.hh"
#include "manager.hh"

namespace gazebo {

    class GAZEBO_VISIBLE sdcCar : public ModelPlugin {
        // Constructor for sdcCar
        public: sdcCar();

        // These methods are called by Gazebo during the loading and initializing
        // stages of world building and populating
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();
        
        // Bound to Gazebo's world update, gets called every tick of the simulation
        private: void OnUpdate();
        
         // Holds the bound connection to Gazebo's update, necessary in order to properly
        // receive updates
        std::vector<event::ConnectionPtr> connections;

        // The Gazebo model representation of the car
        physics::ModelPtr model;
        // Contains the wheel joints that get operated on each tick for movement
        std::vector<physics::JointPtr> joints;
        // A link to the chassis of the car, mainly used for access to physics variables
        // related to the car's state
        physics::LinkPtr chassis;

    };
}
#endif
