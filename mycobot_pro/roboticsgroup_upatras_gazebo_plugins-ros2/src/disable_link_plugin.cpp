/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <roboticsgroup_upatras_gazebo_plugins/disable_link_plugin.hpp>

namespace gazebo {

    DisableLinkPlugin::DisableLinkPlugin()
    {
    }

    DisableLinkPlugin::~DisableLinkPlugin()
    {
    }

    void DisableLinkPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        auto model = _parent;
        auto world = model->GetWorld();

        // Check for link element
        if (!_sdf->HasElement("link")) {
            gzerr << "No link element present. DisableLinkPlugin could not be loaded\n";
            return;
        }

        auto link_name = _sdf->GetElement("link")->Get<std::string>();

        // Get pointers to joints
        auto link = model->GetLink(link_name);
        if (link) {
            link->SetEnabled(false);
            gzmsg << "DisableLinkPlugin loaded! Link: " << link_name << "\n";
        }
        else {
            gzerr << "Link " << link_name << " not found! DisableLinkPlugin could not be loaded\n";
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(DisableLinkPlugin);

}  // namespace gazebo
