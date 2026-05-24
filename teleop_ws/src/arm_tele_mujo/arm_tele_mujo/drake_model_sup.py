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
from pydrake.multibody import inverse_kinematics
# from pydrake.visualization import AddFrameTriadIllustration
from launch_ros.substitutions import FindPackageShare

shoulder_to_base = RotationMatrix.MakeXRotation(- np.pi / 2) @ RotationMatrix.MakeZRotation(np.pi / 2)
elbow_to_base = RotationMatrix.MakeXRotation(- np.pi / 2) @ RotationMatrix.MakeZRotation(np.pi / 2)
wrist_to_base = RotationMatrix.MakeXRotation(- np.pi / 2) @ RotationMatrix.MakeZRotation(np.pi / 2)

Conju_trans = [wrist_to_base,elbow_to_base,shoulder_to_base]

class arm_ik():
    def __init__(self,model_path,arm_type:str):
        self.model_path = model_path
        self.arm_type = arm_type
        self.q_last = None
        self.arm_model_url = None
        self.plant = self.load_robot()
        self.context = self.plant.CreateDefaultContext()

        self.plant_ad = self.plant.ToAutoDiffXd()
        self.context_ad = self.plant_ad.CreateDefaultContext()

        self._q = np.zeros(44)

        self.up_zero_R,self.elbow_zero_R, self.wrist_1_zero_R, self.wrist_2_zero_R = self.get_bias()
        # print(self.up_zero_R)
        # print(self.elbow_zero_R)
        # print(self.wrist_zero_R)


    def load_robot(self):

        self.arm_model_url = os.path.join(FindPackageShare('arm_hand_description').find('arm_hand_description'),f'{self.model_path}')

        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant).AddModelsFromUrl(
            f'file://{self.arm_model_url}'
        )
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))
        plant.Finalize()
        return plant
    
    def get_bias(self):
        self.plant.SetPositions(self.context, np.zeros(44))
        _Base = self.plant.GetFrameByName("base_link")
        _UP_Arm = self.plant.GetFrameByName("seg1_link4")
        _Forearm = self.plant.GetFrameByName("seg2_link4")
        _Wrist_1 = self.plant.GetFrameByName("seg3_link2")
        _Wrist_2 = self.plant.GetFrameByName("seg4_link4")
        up_zero_R = self.plant.CalcRelativeRotationMatrix(self.context, _Base, _UP_Arm)
        elbow_zero_R = self.plant.CalcRelativeRotationMatrix(self.context, _UP_Arm, _Forearm)
        wrist_1_zero_R = self.plant.CalcRelativeRotationMatrix(self.context, _Forearm, _Wrist_1)
        wrist_2_zero_R = self.plant.CalcRelativeRotationMatrix(self.context,_Wrist_1,_Wrist_2)

        return up_zero_R, elbow_zero_R, wrist_1_zero_R, wrist_2_zero_R


    def visual(self,position,meshcat):
        visualizer = ModelVisualizer(visualize_frames=True,meshcat=meshcat)
        visualizer.parser().AddModelsFromUrl(f'file://{self.arm_model_url}')
        visualizer._builder.plant().WeldFrames(visualizer._builder.plant().world_frame(), visualizer._builder.plant().GetFrameByName("base_link"))
        visualizer.Run(position=position)
    
    def ori_inv_sup(self,up_ori = None,elbow_ori = None,wrist_ori = None,q_last = None):
        q_nominal = q_last

        self.plant.SetPositions(self.context, q_nominal)
        if up_ori is None:
            up_ori = np.eye(3)
        up_ori = RotationMatrix(up_ori)
        # print(up_ori)

        if elbow_ori is None:
            elbow_ori = np.eye(3)
        elbow_ori = RotationMatrix(elbow_ori)
        if wrist_ori is None:
            wrist_ori = np.eye(3)
        wrist_ori = RotationMatrix(wrist_ori)
        # print(wrist_ori.ToRollPitchYaw().vector())

        _Base = self.plant.GetFrameByName("base_link")
        _UP_Arm = self.plant.GetFrameByName("seg1_link4")
        _Forearm = self.plant.GetFrameByName("seg2_link4")
        _Wrist_1 = self.plant.GetFrameByName("seg3_link2")
        _Wrist_2 = self.plant.GetFrameByName("seg4_link4")
        _W = self.plant.world_frame()
        sup_arm = self.plant.GetModelInstanceByName("sup_manipulator")

        axis_W = wrist_ori.ToAngleAxis().axis()
        angle_W = wrist_ori.ToAngleAxis().angle()
        # print(axis_W,angle_W)

        angle_W_1 = angle_W * 0.7
        angle_W_2 = angle_W - angle_W_1

        wrist_ori_1 = RotationMatrix(AngleAxis(angle_W_1,axis_W))
        wrist_ori_2 = RotationMatrix(AngleAxis(angle_W_2,axis_W))

        print(wrist_ori_1,wrist_ori_2)
        
        up_ori, q_up = self.Calc_manifold_map((up_ori @ self.up_zero_R).matrix(),base_frame='base_link',rela_frame='seg1_link4')
        elbow_ori, q_elbow = self.Calc_manifold_map((elbow_ori @ self.elbow_zero_R).matrix(),base_frame='seg1_link4',rela_frame='seg2_link4')
        # wrist_ori, q_wrist = self.Calc_manifold_map((wrist_ori @ self.wrist_zero_R).matrix(),base_frame='seg2_link4',rela_frame='seg4_link4')
        wrist_ori_1,q_wrist_1 = self.Calc_manifold_map((wrist_ori_1 @ self.wrist_1_zero_R).matrix(),base_frame='seg2_link4', rela_frame='seg3_link2')
        wrist_ori_2,q_wrist_2 = self.Calc_manifold_map((wrist_ori_2 @ self.wrist_2_zero_R).matrix(),base_frame='seg3_link2', rela_frame='seg4_link4')

        result_q = np.concatenate((q_up[0:16],q_elbow[16:32],q_wrist_1[32:40],q_wrist_2[40:44]))
        self._q = result_q
        return result_q
    
    def CalcRotMetric(self, R_3dim: np.matrix, R_2dim: np.matrix):
        trace = (R_3dim.T @ R_2dim).trace()
        

        val = (trace - 1) / 2.0
        
        # 将值严格限制在 -1.0 到 1.0 之间，防止数学定义域报错
        # 注意：Drake 的优化器通常传入的是带有梯度的 AutoDiffXd 类型对象。
        # 直接使用普通的 if/else 或 numpy.clip 可能会导致梯度链断裂。
        
        if hasattr(val, 'value'): # 检查输入是否为 AutoDiffXd 类型
            from pydrake.math import min, max
            # 使用 Drake 原生的数学函数，确保梯度图完好无损
            val_clamped = max(-1.0, min(1.0, val))
        else:
            # 如果是普通的 float 类型，直接使用 numpy 的 clip
            val_clamped = np.clip(val, -1.0, 1.0)

        # geo_dist = arccos(val_clamped)
        geo_dist = 3.0 - val_clamped

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
    
    def AddmimicConstrain(self,opt_prog,q_var):
        sup_arm = self.plant.GetModelInstanceByName("sup_manipulator")
        mimic_groups = [
            [f"seg1_x{i}" for i in range(1, 9)],
            [f"seg1_y{i}" for i in range(1, 9)],
            [f"seg2_x{i}" for i in range(1, 9)],
            [f"seg2_y{i}" for i in range(1, 9)],
            [f"seg3_x{i}" for i in range(1, 5)],
            [f"seg3_y{i}" for i in range(1, 5)],
            [f"seg4_x{i}" for i in range(1, 3)],
            [f"seg4_y{i}" for i in range(1, 3)],
        ]

        for group in mimic_groups:
            joint_0 = self.plant.GetJointByName(group[0], sup_arm)
            idx_0 = joint_0.position_start()
            var_0 = q_var[idx_0]  # 注意：这里使用的是 opt_prog 的变量 q_var
            
            for i in range(1, len(group)):
                joint_i = self.plant.GetJointByName(group[i], sup_arm)
                idx_i = joint_i.position_start()
                var_i = q_var[idx_i]
                
                # 为流形优化的数学程序添加等式约束
                opt_prog.AddLinearEqualityConstraint(var_0 - var_i, 0)
    
    def Calc_manifold_map(self,R_3dim:np.matrix,base_frame:str,rela_frame:str):

        opt_prog = MathematicalProgram()
        q_var = opt_prog.NewContinuousVariables(self.plant.num_positions())
        cost_func = partial(
            self.CalcFrameMetric,
            R_3dim =R_3dim,
            base_frame = base_frame,rela_frame = rela_frame,
            )
        self.AddmimicConstrain(opt_prog = opt_prog,q_var = q_var)
        

        ################################## make sure the cost_func signature is correct ####################################
        # if True:
        #     # print('Checking cost_func signature')
        #     signature = inspect.signature(cost_func)
        #     unbound_params = [param for param in signature.parameters.values() if param.default is inspect.Parameter.empty]
        #     assert len(unbound_params) == 1 and unbound_params[0].name == 'q_var', "cost_func must have only one unbound parameter"

        ############################################################################################################

        opt_prog.AddCost(cost_func,vars=q_var)

        weight = 1e-4
        Q = weight * np.eye(self.plant.num_positions())
        opt_prog.AddQuadraticErrorCost(Q, self._q, q_var)

        result = Solve(opt_prog)
        if not result.is_success():
            solver_name = result.get_solver_id().name()
            fail_reason = result.get_solution_result()
            print(f"Optimization failed! Solver: {solver_name}, Reason: {fail_reason}")
            return None

        q_sol = result.GetSolution(q_var)
        print(f'result:{rela_frame}:{q_sol}')
        self.plant.SetPositions(self.context,q_sol)
        rot_res = self.plant.CalcRelativeRotationMatrix(self.context,self.plant.GetFrameByName(base_frame),self.plant.GetFrameByName(rela_frame))
        # print(f"q_sol: {q_sol})")
        return rot_res,q_sol
        
    def record_rot_data(self):
        self.plant.SetPositions(self.context,self._q)


        _G = self.plant.GetFrameByName("base_link")
        _G1 = self.plant.GetFrameByName("seg1_link4")
        _G2 = self.plant.GetFrameByName("seg2_link4")
        _G3 = self.plant.GetFrameByName("seg4_link4")

        R_s = self.plant.CalcRelativeRotationMatrix(self.context,_G,_G1).matrix()
        R_e = self.plant.CalcRelativeRotationMatrix(self.context,_G1,_G2).matrix()
        R_w = self.plant.CalcRelativeRotationMatrix(self.context,_G2,_G3).matrix()
        T = self.plant.CalcRelativeTransform(self.context,_G,_G3)
        return R_s,R_e,R_w,T

def Quatnumpy_to_Rotation(q: np.ndarray) -> List[np.ndarray]:
    
    # assert q.shape == (16,), "Input shape must be (16,)"
    rots = []

    quats = q[0:12].reshape(-1, 4)  # shape(3, 4)
    ###  Conjugation transformation
    for i,quat in enumerate(quats):
        rot = Quaternion(quat)
        rot = Conju_trans[i].transpose() @ RotationMatrix(rot) @ Conju_trans[i]
        rots.append(rot)
    return rots



def main():
    # 1. 启动 Meshcat 可视化服务器
    meshcat = StartMeshcat()
    
    # 2. 实例化 arm_ik 类
    # 请确保 'sup_arm_description/urdf/sup_arm.sdf' 替换为你实际的相对路径
    # 该路径会与 FindPackageShare('arm_hand_description') 进行拼接
    model_relative_path = 'sup_arm/urdf/sup_arm.sdf' # 根据你的实际工程目录修改
    arm = arm_ik(model_relative_path, arm_type='sup_arm')
    
    # 3. 构造初始猜测关节角度 (q_last)
    # 根据你的类定义，sup_arm 有 44 个自由度
    q_initial = np.zeros(44)
    
    # 4. 构造测试目标姿态 (Target Orientations)
    # 这里我们构造几个简单的绕坐标轴旋转的姿态作为测试输入
    print("正在生成目标姿态...")
    up_target = RotationMatrix()
    elbow_target = RotationMatrix()
    wrist_target = RotationMatrix()
    up_target =   RotationMatrix.MakeXRotation(np.pi / 3)      # 大臂目标：绕 X 轴转 30 度
    elbow_target = Conju_trans[1].transpose() @ RotationMatrix.MakeYRotation( - np.pi / 6) @ Conju_trans[1] # 小臂目标：绕 Y 轴转 30 度
    wrist_target = Conju_trans[2].transpose() @ RotationMatrix.MakeYRotation(  np.pi / 6) @ Conju_trans[2] # 手腕目标：绕 X 轴转 60 度
    
    # 5. 调用逆运动学求解器
    print("开始求解逆运动学 (IK)...")
    start_time = time.time()
    
    res_q = arm.ori_inv_sup(
        up_ori=up_target.matrix(), 
        elbow_ori=elbow_target.matrix(), 
        wrist_ori=wrist_target.matrix(), 
        q_last=q_initial
    )
    
    end_time = time.time()
    print(f"求解耗时: {end_time - start_time:.4f} 秒")
    

    # res_q = np.zeros(44)
    # 6. 处理结果并进行可视化
    if res_q is not None:
        print("✅ IK 求解成功！")

        print(f"求解结果: {res_q}")
        
        arm.plant.SetPositions(arm.context,res_q)

        print("正在将结果发送至 Meshcat 进行可视化...")
        print("请点击上方终端输出的 Meshcat URL 查看三维模型。")
        # 调用类内部的可视化方法
        arm.visual(position=res_q, meshcat=meshcat)

        # 保持程序运行，以便在浏览器中观察模型
        input("按 Enter 键退出程序...")
    else:
        print("❌ IK 求解失败。请检查目标姿态是否超出了机械臂的物理可达工作空间（或流形映射失败）。")

if __name__ == '__main__':
    main()

