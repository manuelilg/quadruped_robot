/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_QUADRUPED_SPIDER_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_QUADRUPED_SPIDER_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot spider.
 */
class QuadrupedSpiderKinematicModel : public KinematicModel {
public:
  QuadrupedSpiderKinematicModel () : KinematicModel(4)
  {
    // x/y min 0.06

    const double x_nominal_b = 0.08;
    const double y_nominal_b = 0.1; // TODO
    const double z_nominal_b = -0.055; // TODO

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    // +/- for each leg -> nominal_stance has to be at the center of the whole move range
    max_dev_from_nominal_ << 0.05, 0.03, 0.03;
  }
};

/**
 * @brief The Dynamics of the quadruped robot spider.
 */
class QuadrupedSpiderDynamicModel : public SingleRigidBodyDynamics {
public:
  QuadrupedSpiderDynamicModel()
  : SingleRigidBodyDynamics(0.335353, //mass
                    // Ixx       Iyy          Izz         Ixy         Ixz         Iyz
                    0.000825508, 0.000785667, 0.00142973, -5.596e-07, 1.3094e-06, 2.99247e-06,
                    4) {}
};

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_QUADRUPED_SPIDER_MODEL_H_ */
