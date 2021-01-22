/*
 * Copyright 2021 Technology Innovation Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef _GAZEBO_PLUGINS_GAZEBO_GROUP_POS_PLUGIN_H
#define _GAZEBO_PLUGINS_GAZEBO_GROUP_POS_PLUGIN_H

#include <string>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sys/socket.h>
#include <netinet/in.h>

namespace gazebo {


/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboGroupPosPlugin : public WorldPlugin {
public:
    GazeboGroupPosPlugin()
    : WorldPlugin()
    {}

    virtual ~GazeboGroupPosPlugin();

protected:

    void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
    void OnUpdate(const common::UpdateInfo& /*_info*/);

private:
    void GetModelPositions();

    transport::NodePtr node_;

    event::ConnectionPtr conn_;
    physics::WorldPtr world_;

    double update_rate_;
    double update_interval_;
    common::Time last_update_;

    std::string name_prefix_;

    int sock_;
    struct sockaddr_in addr_;
    std::string udp_address_;
    uint16_t udp_port_;
};

} // namespace gazebo

#endif // _GAZEBO_PLUGINS_GAZEBO_GROUP_POS_PLUGIN_H
