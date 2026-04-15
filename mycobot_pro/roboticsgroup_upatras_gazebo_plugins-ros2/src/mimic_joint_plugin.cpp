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

#include <rclcpp/duration.hpp>

#include <roboticsgroup_upatras_gazebo_plugins/mimic_joint_plugin.hpp>

namespace math = ignition::math;

namespace gazebo {

    MimicJointPlugin::MimicJointPlugin()
    {
        joint_.reset();
        mimic_joint_.reset();
    }

    MimicJointPlugin::~MimicJointPlugin()
    {
        update_connection_.reset();
    }

    void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        world_ = _parent->GetWorld();

        // NOTE Namespace should be set within the <ros> tag of the plugin sdf
        ros_node_ = gazebo_ros::Node::Get(_sdf);

        // Check for joint element
        if (!_sdf->HasElement("joint")) {
            RCLCPP_ERROR_STREAM(
                ros_node_->get_logger(),
                "Failed to load MimicJointPlugin: no joint element present");
            return;
        }
        auto joint_name = _sdf->GetElement("joint")->Get<std::string>();

        // Check for mimicJoint element
        if (!_sdf->HasElement("mimicJoint")) {
            RCLCPP_ERROR_STREAM(
                ros_node_->get_logger(),
                "Failed to load MimicJointPlugin: no mimicJoint element present");
            return;
        }
        auto mimic_joint_name = _sdf->GetElement("mimicJoint")->Get<std::string>();

        // Check if PID controller wanted
        if (_sdf->HasElement("hasPID")) {
            auto name = _sdf->GetElement("hasPID")->Get<std::string>();
            if (name.empty()) {
                name = "gazebo_ros_control/pid_gains/" + mimic_joint_name;
            }
            pid_ = std::make_unique<control_toolbox::PidROS>(ros_node_, "name");
            pid_->initPid();
        }

        // Check for multiplier element
        multiplier_ = 1.0;
        if (_sdf->HasElement("multiplier")) {
            multiplier_ = _sdf->GetElement("multiplier")->Get<double>();
        }

        // Check for offset element
        offset_ = 0.0;
        if (_sdf->HasElement("offset")) {
            offset_ = _sdf->GetElement("offset")->Get<double>();
        }

        // Check for sensitiveness element
        sensitiveness_ = 0.0;
        if (_sdf->HasElement("sensitiveness")) {
            sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();
        }

        // Get pointers to joints
        joint_ = _parent->GetJoint(joint_name);
        if (!joint_) {
            RCLCPP_ERROR_STREAM(
                ros_node_->get_logger(),
                "Failed to load MimicJointPlugin: no " << joint_name << " joint found");
            return;
        }
        mimic_joint_ = _parent->GetJoint(mimic_joint_name);
        if (!mimic_joint_) {
            RCLCPP_ERROR_STREAM(
                ros_node_->get_logger(),
                "Failed to load MimicJointPlugin: no " << mimic_joint_name << " (mimic) joint found");
            return;
        }

        // Check for max effort
        max_effort_ = mimic_joint_->GetEffortLimit(0);
        if (_sdf->HasElement("maxEffort")) {
            max_effort_ = _sdf->GetElement("maxEffort")->Get<double>();
        }

        // Set max effort
        if (!pid_) {
            mimic_joint_->SetParam("fmax", 0, max_effort_);
        }

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&MimicJointPlugin::UpdateChild, this));

        RCLCPP_ERROR_STREAM(
            ros_node_->get_logger(),
            "MimicJointPlugin loaded! Joint: \"" << joint_name << "\", Mimic joint: \"" << mimic_joint_name << "\""
                                                               << ", Multiplier: " << multiplier_ << ", Offset: " << offset_
                                                               << ", MaxEffort: " << max_effort_ << ", Sensitiveness: " << sensitiveness_);
    }

    void MimicJointPlugin::UpdateChild()
    {
        static auto period(rclcpp::Duration::from_seconds(world_->Physics()->GetMaxStepSize()));

        // Set mimic joint's angle based on joint's angle
        double angle = joint_->Position(0) * multiplier_ + offset_;
        double a = mimic_joint_->Position(0);

        if (fabs(angle - a) >= sensitiveness_) {
            if (pid_) {
                if (a != a)
                    a = angle;
                double error = angle - a;
                double effort = pid_->computeCommand(error, period);
                effort = math::clamp(effort, -max_effort_, max_effort_);
                mimic_joint_->SetForce(0, effort);
            }
            else {
                mimic_joint_->SetPosition(0, angle, true);
            }
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin);

}  // namespace gazebo
