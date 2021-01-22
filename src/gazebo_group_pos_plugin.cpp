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

#include "gazebo_group_pos_plugin.h"
#include "common.h"

namespace gazebo {

GazeboGroupPosPlugin::~GazeboGroupPosPlugin() {
    conn_->~Connection();
}

void GazeboGroupPosPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {

    world_ = world;
    node_ = transport::NodePtr(new transport::Node());
    node_->Init(world_->Name());

    update_rate_ = 1.0;
    if (sdf->HasElement("updateRate")) {
        update_rate_ = sdf->GetElement("updateRate")->Get<double>();
    }
    update_interval_ = (update_rate_ > 0.0) ? 1/update_rate_ : 0.0;

    name_prefix_ = "ssrc_fog_x-";
    if (sdf->HasElement("namePrefix")) {
        name_prefix_ = sdf->GetElement("namePrefix")->Get<std::string>();
    }

    udp_address_ = "127.0.0.1";
    if (sdf->HasElement("udpAddress")) {
        udp_address_ = sdf->GetElement("udpAddress")->Get<std::string>();
    }

    udp_port_ = 15517;
    if (sdf->HasElement("udpPort")) {
        udp_port_ = (uint16_t) sdf->GetElement("udpPort")->Get<int>();
    }

    conn_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboGroupPosPlugin::OnUpdate, this, _1));

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        gzerr << "ERROR opening socket: " << sock_ << "\n";
        return;
    }

    int broadcast = 1;
    if (setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
    {
        gzerr << "ERROR setting Broadcast option\n";
        close(sock_);
        return;
    }

    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(udp_port_);
    addr_.sin_addr.s_addr = inet_addr(udp_address_.c_str());
#if GAZEBO_MAJOR_VERSION >= 9
    last_update_ = world_->SimTime();
#else
    last_update_ = world_->GetSimTime();
#endif

}

void GazeboGroupPosPlugin::GetModelPositions() {
    unsigned int count = world_->ModelCount();
    if (count == 0)
        return;

    unsigned int drone_count = 0;
    std::string message;
    for(int i=0; i<count; i++) {
        physics::ModelPtr m = world_->ModelByIndex(i);
        if (m) {
            std::string name = m->GetName();
            if (name.compare((size_t) 0, (size_t) name_prefix_.length(), name_prefix_.c_str()) == 0) {
                name.erase(0, name_prefix_.length());
                ++drone_count;
                ignition::math::Pose3d pos = m->WorldPose();
                if (!message.empty())
                    message += ", ";
                message += "{ \"name\": \"" + name +
                    "\", \"x\": \"" + std::to_string(pos.X()) +
                    "\", \"y\": \"" + std::to_string(pos.Y()) +
                    "\", \"z\": \"" + std::to_string(pos.Z()) +
                    "\" }";
            }
        }
    }
    if (!message.empty()) {
        message = "{ \"positions\": [ " + message + " ] }";
        sendto(sock_, message.c_str(),message.length(), 0, (sockaddr *) &addr_, sizeof(addr_));
    }
}

// This gets called by the world update start event.
void GazeboGroupPosPlugin::OnUpdate(const common::UpdateInfo& data) {
    // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
#else
    common::Time now = world_->GetSimTime();
#endif
    if ((now - last_update_).Double() < update_interval_ || update_interval_ == 0.0) {
        return;
    }
    last_update_ = now;

    GetModelPositions();
}

GZ_REGISTER_WORLD_PLUGIN(GazeboGroupPosPlugin);
}
