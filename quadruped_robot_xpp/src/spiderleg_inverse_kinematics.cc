/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

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

#include <xpp_spider/spiderleg_inverse_kinematics.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

#include <ros/ros.h>

namespace xpp {


SpiderlegInverseKinematics::Vector3d
SpiderlegInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B, const int footId) const
{
  //ROS_INFO_STREAM("ee_pos_B: \n"  << ee_pos_B);


  // translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg
  Eigen::Vector3d xr = ee_pos_B;

  // compute the HAA angle
  //const double q_HAA = atan2(xr[X], xr[Y]);
  // GAGA
  // HHAG
  // sct
  //ROS_INFO_STREAM("xr org: \n" << xr);
  const double rot = atan2(xr[Y], xr[X]);
  //ROS_INFO_STREAM("rot: \n" << rot);
  double q_HAA_temp;
  if (footId == quad::LF || footId == quad::RH) {
    q_HAA_temp = M_PI/2 - rot;
  }
  else {
    q_HAA_temp = rot;
  }
  const double q_HAA = q_HAA_temp;

  // spider: rotate into HFE coordinate system (rot around Z)
  Eigen::Matrix3d R;
  R << cos(-rot), -sin(-rot), 0.0,  sin(-rot), cos(-rot), 0.0,  0.0, 0.0, 1.0;
  xr = (R * xr).eval();

  xr -= haa_to_hfe; // distance of HAA to HFE in y and z direction
  //ROS_INFO_STREAM("xr: \n" << xr);

  // compute square of length from HFE to foot
  const double tmp1 = pow(xr[X],2)+pow(xr[Z],2);

  // compute temporary angles (with reachability check)
  //ROS_INFO_STREAM("xr[Z]: " << xr[Z]);
  //ROS_INFO_STREAM("sqrt(tmp1): " << sqrt(tmp1));
  const double alpha = asin(xr[Z] / sqrt(tmp1));    //G/H = sin, G:xr[Z], H:tmp1,
  // alpha should be negative (when xr[Z] is negative)
  //ROS_INFO_STREAM("alpha: " << alpha);
  // cos beta = (a^2 + c^2 - b^2) / 2*a*c, a: length_thigh, b: length_shank, c^2:tmp1
  const double beta = acos((pow(length_thigh, 2) + tmp1 - pow(length_shank, 2)) / (2*length_thigh*sqrt(tmp1)));
  //ROS_INFO_STREAM("beta: " << beta);

  double q_HFE_temp;
  if (footId == quad::LF || footId == quad::RH) {
    q_HFE_temp = alpha + beta;
  }
  else {
    q_HFE_temp = -(alpha + beta);
  }
  const double q_HFE = q_HFE_temp + M_PI/2;

  // cos gamma = (a^2 + b^2 - c^2) / 2*a*b, a: length_thigh, b: length_shank, c^2:tmp1
  const double gamma = acos((pow(length_thigh, 2) + pow(length_shank, 2) - tmp1) / (2 * length_thigh * length_shank));

  double q_KFE_temp;
  if (footId == quad::LF || footId == quad::RH) {
    q_KFE_temp = gamma;
  }
  else {
    q_KFE_temp = M_PI - gamma;
  }
  const double q_KFE = q_KFE_temp;

  // forward knee bend
//  EnforceLimits(q_HAA_bf, HAA);
//  EnforceLimits(q_HFE_bf, HFE);
//  EnforceLimits(q_KFE_bf, KFE);

  return Vector3d(q_HAA, q_HFE, q_KFE);
//  Vector3d returnValue = Vector3d(q_HAA, q_HFE, q_KFE);
//  ROS_INFO_STREAM("Joint Angles: \n" << returnValue);
//  return returnValue;
}

void
SpiderlegInverseKinematics::EnforceLimits (double& val, SpiderJointID joint) const
{
  // totally exaggerated joint angle limits
  const static double haa_min = -360;
  const static double haa_max =  360;

  const static double hfe_min = -360;
  const static double hfe_max =  360;

  const static double kfe_min = -360;
  const static double kfe_max =  360;

  // reduced joint angles for optimization
  static const std::map<SpiderJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<SpiderJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

} /* namespace xpp */
