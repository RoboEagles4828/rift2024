# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import Optional
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView
import torch

class EdnaView(ArticulationView):
    def __init__(
        self,
        prim_paths_expr: str,
        name: Optional[str] = "EdnaView",
    ) -> None:
        """[summary]
        """

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
            reset_xform_properties=False
        )
        
        self._base = RigidPrimView(prim_paths_expr="/World/envs/.*/edna/swerve_chassis_link", name="base_view", reset_xform_properties=False)

        self._axle = [
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/front_left_axle_link", name="axle_view[0]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/front_right_axle_link", name="axle_view[1]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/rear_left_axle_link", name="axle_view[2]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/rear_right_axle_link", name="axle_view[3]", reset_xform_properties=False)
            ]
        self._wheel = [
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/front_left_wheel_link", name="wheel_view[0]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/front_right_wheel_link", name="wheel_view[1]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/rear_left_wheel_link", name="wheel_view[2]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/edna/rear_right_wheel_link", name="wheel_view[3]", reset_xform_properties=False)
            ]
        
    def get_axle_positions(self):
        
        axle_pose1, __ = self._axle[0].get_local_poses()
        axle_pose2, __ = self._axle[1].get_local_poses()
        axle_pose3, __ = self._axle[2].get_local_poses()
        axle_pose4, __ = self._axle[3].get_local_poses()
                
        tuple = (torch.transpose(axle_pose1, 0, 1), 
                 torch.transpose(axle_pose2, 0, 1), 
                 torch.transpose(axle_pose3, 0, 1), 
                 torch.transpose(axle_pose4, 0, 1)
                )        
        tuple_tensor = torch.cat(tuple)
        return torch.transpose(tuple_tensor, 0, 1)
