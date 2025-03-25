import os
import numpy as np
from typing import List
import time
from functools import partial
import inspect

from pydrake.all import (
    MultibodyPlant,
    Parser,
    StartMeshcat,
    RotationMatrix,
    Quaternion,
    RollPitchYaw,
    Solve,
    ModelVisualizer,
    AngleAxis,
    Frame,
    MathematicalProgram,
)
from pydrake.math import arccos
from pydrake.all import AutoDiffXd,ExtractGradient,ExtractValue
from pydrake.multibody import inverse_kinematics
# from pydrake.visualization import AddFrameTriadIllustration
from launch_ros.substitutions import FindPackageShare

class arm_ik():
    def __init__(self,model_path,arm_type:str):
        self.model_path = model_path
        self.arm_type = arm_type
        self.q_last = None
        self.arm_model_url = None
        self.amend_frame = RotationMatrix().MakeZRotation(np.pi/2)@RotationMatrix.MakeXRotation(np.pi/2) \
            if 'UR5' in arm_type else RotationMatrix()
        self.plant = self.load_robot()
        self.context = self.plant.CreateDefaultContext()

        self.plant_ad = self.plant.ToAutoDiffXd()
        self.context_ad = self.plant_ad.CreateDefaultContext()

        self._q = np.zeros(7) \
            if '_7dof' in arm_type else np.zeros(6)

    def load_robot(self):

        self.arm_model_url = os.path.join(FindPackageShare('arm_hand_description').find('arm_hand_description'),f'{self.model_path}')

        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant).AddModelsFromUrl(
            f'file://{self.arm_model_url}'
        )

        if 'iiwa14' in self.arm_type:
            plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))

        elif 'UR5' in  self.arm_type:
            plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))

        elif 'roake' in self.arm_type:
            plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("xMatePro3_base"))

        plant.Finalize()
        return plant
    
    def visual(self,position,meshcat):
        visualizer = ModelVisualizer(visualize_frames=True,meshcat=meshcat)
        visualizer.parser().AddModelsFromUrl(f'file://{self.arm_model_url}')
        visualizer._builder.plant().WeldFrames(visualizer._builder.plant().world_frame(), visualizer._builder.plant().GetFrameByName("base_link"))
        visualizer.Run(position=position)
    
    def ori_inv(self,up_ori = None,elbow_ori = None,wrist_ori = None,q_last = None):
        
        if q_last is not None:
            q_nominal = q_last
        else:
            q_nominal = np.array([0.2, 0.4, 0.6, 0.0, -0.3, -0.0,0.7]) \
            if '_7dof' in self.arm_type else np.array([0.0, 0.0, 0.0, 0.0, -0.0, 0.0])

        self.plant.SetPositions(self.context, q_nominal)

        if up_ori is None:
            up_ori = np.array([1,0,0,0])
        up_ori = RotationMatrix(Quaternion(up_ori))
        if elbow_ori is None:
            elbow_ori = np.array([1,0,0,0])
        elbow_ori = RotationMatrix(Quaternion(elbow_ori))
        if wrist_ori is None:
            wrist_ori = np.array([1,0,0,0])
        wrist_ori = RotationMatrix(Quaternion(wrist_ori))

        if 'iiwa14' in self.arm_type:
            _Base = self.plant.GetFrameByName("iiwa_link_0")
            _UP_Arm = self.plant.GetFrameByName("iiwa_link_3")
            _Forearm = self.plant.GetFrameByName("iiwa_link_4")
            _Wrist = self.plant.GetFrameByName("iiwa_link_7")
            _W = self.plant.world_frame()
            iiwa = self.plant.GetModelInstanceByName("iiwa14")

        elif 'roake' in self.arm_type:
            _Base = self.plant.GetFrameByName("xMatePro3_base")
            _UP_Arm = self.plant.GetFrameByName("xMatePro3_link3")
            _Forearm = self.plant.GetFrameByName("xMatePro3_link4")
            _Wrist = self.plant.GetFrameByName("xMatePro3_link7")
            _W = self.plant.world_frame()
            roake = self.plant.GetModelInstanceByName("xMatePro3")


        elif 'UR5' in self.arm_type :
            _Base = self.plant.GetFrameByName("base_link")
            _UP_Arm = self.plant.GetFrameByName("upper_arm_link")
            _Forearm = self.plant.GetFrameByName("forearm_link")
            _Wrist = self.plant.GetFrameByName("wrist_3_link")
            _W = self.plant.world_frame()
            UR5 = self.plant.GetModelInstanceByName("ur5")
            up_ori = self.Calc_manifold_map(up_ori.matrix(),base_frame='base_link',rela_frame='upper_arm_link')
            # arm_start = self.plant.GetJointByName("shoulder_pan_joint",UR5).velocity_start()
            # arm_end = self.plant.GetJointByName("wrist_3_joint",UR5).velocity_start()


        def AddOrientationConstraint(ik, R_WG, bounds,base_frame,rela_frame):
            """Add orientation constraint to the ik problem. Implements an inequality
            constraint where the axis-angle difference between f_R(q) and R_WG must be
            within bounds. Can be translated to:
            ik.prog().AddBoundingBoxConstraint(angle_diff(f_R(q), R_WG), -bounds, bounds)
            """
            ik.AddOrientationConstraint(
                frameAbar=base_frame,
                R_AbarA=R_WG,
                frameBbar=rela_frame,
                R_BbarB=RotationMatrix(),
                theta_bound=bounds,
            )

        # def AddPositionConstraint(ik, p_WG_lower, p_WG_upper,base_frame,rela_frame):
        #     """Add position constraint to the ik problem. Implements an inequality
        #     constraint where f_p(q) must lie between p_WG_lower and p_WG_upper. Can be
        #     translated to
        #     ik.prog().AddBoundingBoxConstraint(f_p(q), p_WG_lower, p_WG_upper)
        #     """
        #     ik.AddPositionConstraint(
        #         frameA=base_frame,
        #         frameB=rela_frame,
        #         p_BQ=np.zeros(3),
        #         p_AQ_lower=p_WG_lower,
        #         p_AQ_upper=p_WG_upper,
        #     )

        orientation_bounds = 0.05
        # position_bounds = np.array([0.0, 0.0, 0.0]) 
        ik = inverse_kinematics.InverseKinematics(self.plant)
        q_variables = ik.q()
        prog = ik.prog()

        prog.SetInitialGuess(q_variables,q_nominal)

        assert up_ori is not None ,"up_ori is None"
        if up_ori is not None:
            up_WG = up_ori
            AddOrientationConstraint(ik, up_WG, orientation_bounds,base_frame=_Base,rela_frame=_UP_Arm)
        
        elbow_WG_rela = up_WG.inverse() @ elbow_ori
        # print(f'{elbow_WG_rela.ToRollPitchYaw()}')
        elbow_angle = - elbow_WG_rela.ToRollPitchYaw().pitch_angle()

        if wrist_ori is not None:
            wrist_WG = wrist_ori@self.amend_frame
            AddOrientationConstraint(ik, wrist_WG, orientation_bounds,base_frame=_Base,rela_frame=_Wrist)

        # ik.prog().AddCost(np.linalg.norm(ik.q() - q_nominal))

        result = Solve(prog)
        if not result.is_success():
            print('IK failed')
            return None
        # assert result.is_success()
        result_q = result.GetSolution(q_variables)
        # print(f"result_q: {result_q}")
        if 'iiwa14' in self.arm_type or 'roake' in self.arm_type:
            result_q[3] = elbow_angle
        elif 'UR5' in self.arm_type:
            result_q[2] = -elbow_angle
            
        # if np.linalg.norm(result_q - q_nominal) > 0.5:
        #     return None
        self._q = result_q
        return result_q
    
    def CalcRotMetric(self,R_3dim:np.matrix,R_2dim:np.matrix):

        trace = (R_3dim.T@R_2dim).trace()
        # geo_dist = norm(logm(R_3dim.T@R_2dim))
        geo_dist = arccos((trace-1)/2)

        return geo_dist
    
    def CalcFrameMetric(self,q_var,R_3dim:np.matrix,base_frame:str,rela_frame:str):
        if q_var.dtype == float:
            plant = self.plant
            context = self.context
        else:
            plant = self.plant_ad
            context = self.context_ad

        plant.SetPositions(context,q_var)
        
        base_frame = plant.GetFrameByName(base_frame)
        rela_frame = plant.GetFrameByName(rela_frame)

        R_2dim = plant.CalcRelativeRotationMatrix(context,base_frame,rela_frame).matrix()

        geo_dist = self.CalcRotMetric(R_3dim,R_2dim)

        return geo_dist
    
    def Calc_manifold_map(self,R_3dim:np.matrix,base_frame:str,rela_frame:str):

        opt_prog = MathematicalProgram()
        q_var = opt_prog.NewContinuousVariables(self.plant.num_positions())
        cost_func = partial(
            self.CalcFrameMetric,
            R_3dim =R_3dim,
            base_frame = base_frame,rela_frame = rela_frame,
            )

        ################################## make sure the cost_func signature is correct ####################################
        # if True:
        #     # print('Checking cost_func signature')
        #     signature = inspect.signature(cost_func)
        #     unbound_params = [param for param in signature.parameters.values() if param.default is inspect.Parameter.empty]
        #     assert len(unbound_params) == 1 and unbound_params[0].name == 'q_var', "cost_func must have only one unbound parameter"

        ############################################################################################################

        opt_prog.AddCost(cost_func,vars=q_var)

        result = Solve(opt_prog)
        if not result.is_success():
            print('Optimization failed')
            return None

        q_sol = result.GetSolution(q_var)
        self.plant.SetPositions(self.context,q_sol)
        rot_res = self.plant.CalcRelativeRotationMatrix(self.context,self.plant.GetFrameByName(base_frame),self.plant.GetFrameByName(rela_frame))
        # print(f"q_sol: {q_sol})")
        return rot_res
        
    def record_rot_data(self):
        self.plant.SetPositions(self.context,self._q)
        if 'iiwa14' in self.arm_type:
            _G = self.plant.GetFrameByName("iiwa_link_0")
            _G1 = self.plant.GetFrameByName("iiwa_link_3")
            _G2 = self.plant.GetFrameByName("iiwa_link_4")
            _G3 = self.plant.GetFrameByName("iiwa_link_7")
        elif 'UR5' in self.arm_type:
            _G = self.plant.GetFrameByName("base_link")
            _G1 = self.plant.GetFrameByName("upper_arm_link")
            _G2 = self.plant.GetFrameByName("forearm_link")
            _G3 = self.plant.GetFrameByName("wrist_3_link")

        elif 'roake' in self.arm_type:
            _G = self.plant.GetFrameByName("xMatePro3_base")
            _G1 = self.plant.GetFrameByName("xMatePro3_link1")
            _G2 = self.plant.GetFrameByName("xMatePro3_link4")
            _G3 = self.plant.GetFrameByName("xMatePro3_link7")

        R_s = self.plant.CalcRelativeRotationMatrix(self.context,_G,_G1).matrix()
        R_e = self.plant.CalcRelativeRotationMatrix(self.context,_G1,_G2).matrix()
        R_w = self.plant.CalcRelativeRotationMatrix(self.context,_G2,_G3).matrix()
        T = self.plant.CalcRelativeTransform(self.context,_G,_G3)
        return R_s,R_e,R_w,T

def Quatnumpy_to_Rotation(q: np.ndarray) -> List[np.ndarray]:
    
    assert q.shape == (16,), "Input shape must be (16,)"
    rots = []

    quats = q.reshape(-1, 4)  # shape(4, 4)

    for quat in quats:
        rot = Quaternion(quat)
        rot = RotationMatrix(rot).ToRollPitchYaw().vector()
        ################## make sure the RollPitchYaw is correct##################
        rot[[1, 2]] = rot[[2, 1]]  
        rot[1] = -rot[1]
        ###########################################################################
        rot = RotationMatrix(RollPitchYaw(rot))
        rots.append(rot)
    return rots



def main():
    meshcat = StartMeshcat()
    arm = arm_ik('UR5/urdf/UR5.sdf',arm_type='UR5')
    # arm = arm_ik('kuka_iiwa_14/urdf/iiwa14.sdf','iiwa14')
    # q = np.zeros(6)
    # q = arm.ori_inv(wrist_ori=[1,0,0,0])

    #######################test manifold_map############################
    Quat = np.array([ 0.6903455, 0.1530459, 0.3799282, 0.5963678 ])
    rot_3 = RotationMatrix(Quaternion(Quat)).matrix()
    rot = arm.Calc_manifold_map(rot_3,'base_link','upper_arm_link')
    print(rot)



