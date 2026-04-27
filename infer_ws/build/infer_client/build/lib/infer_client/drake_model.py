import os
import numpy as np
from typing import List
import time
from functools import partial
import inspect
from rosbags.highlevel import AnyReader
from pathlib import Path
import argparse
import yaml

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
elbow_to_base = RotationMatrix.MakeZRotation(np.pi)
# wrist_to_base = RotationMatrix.MakeYRotation(np.pi)
wrist_to_base = RotationMatrix.MakeXRotation(- np.pi / 2) @ RotationMatrix.MakeZRotation(np.pi / 2)

Conju_trans = [wrist_to_base,elbow_to_base,shoulder_to_base]
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
        
        self.count = 0

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
        

        q_nominal = q_last

        self.plant.SetPositions(self.context, q_nominal)

        if up_ori is None:
            up_ori = np.eye(3)
        up_ori = RotationMatrix(up_ori)
        if elbow_ori is None:
            elbow_ori = np.eye(3)
        elbow_ori = RotationMatrix(elbow_ori)
        if wrist_ori is None:
            wrist_ori = np.eye(3)
        wrist_ori = RotationMatrix(wrist_ori)
        # print(wrist_ori.ToRollPitchYaw().vector())

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
        ik = inverse_kinematics.InverseKinematics(self.plant,with_joint_limits=True)
        q_variables = ik.q()
        prog = ik.prog()

        prog.SetInitialGuess(q_variables,q_nominal)

        assert up_ori is not None ,"up_ori is None"
        if up_ori is not None:
            up_WG = up_ori
            AddOrientationConstraint(ik, up_WG, orientation_bounds,base_frame=_Base,rela_frame=_UP_Arm)
            # print(f'up_ori: {up_WG.ToRollPitchYaw()}')
        
        elbow_angle = elbow_ori.ToRollPitchYaw().roll_angle()

        if wrist_ori is not None:
            wrist_WG = wrist_ori @ self.amend_frame
            AddOrientationConstraint(ik, wrist_WG, orientation_bounds,base_frame=_Base,rela_frame=_Wrist)

        weight = 1e3
        Q = weight * np.eye(len(q_nominal))
        Q_zero = 0.05 * Q
        # b = -Q.dot(q_nominal)
        # prog.AddQuadraticCost(Q,b,q_variables)
        zero_state = np.zeros(len(q_nominal))
        prog.AddQuadraticErrorCost(Q,q_nominal,q_variables)
        prog.AddQuadraticErrorCost(Q_zero,zero_state,q_variables)

        result = Solve(prog)
        if not result.is_success():
            # print('IK failed')
            return None
        # assert result.is_success()
        result_q = result.GetSolution(q_variables)
        # print(f"result_q: {result_q}")
        if 'iiwa14' in self.arm_type :
            result_q[3] = - np.abs(elbow_angle)
        elif 'roake' in self.arm_type:
            result_q[3] = np.abs(elbow_angle)
        elif 'UR5' in self.arm_type:
            result_q[2] =  - np.abs(elbow_angle)
        
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

    

    def calc_and_get_residuals(self, q_sol, original_up_ori, original_wrist_ori):
        """
        计算实际达到的姿态与人类期望姿态之间的投影残差
        q_sol: IK 求解出的当前帧关节角
        original_up_ori: 投影前的人类期望肩部旋转矩阵 (RotationMatrix)
        original_wrist_ori: 投影前的人类期望腕部旋转矩阵 (RotationMatrix)
        """
        # 1. 更新内部状态为当前求解出的关节角
        self.plant.SetPositions(self.context, q_sol)

        # 2. 获取对应的 Frame
        if 'iiwa14' in self.arm_type:
            _Base = self.plant.GetFrameByName("iiwa_link_0")
            _UP_Arm = self.plant.GetFrameByName("iiwa_link_3")
            _Wrist = self.plant.GetFrameByName("iiwa_link_7")
        elif 'UR5' in self.arm_type:
            _Base = self.plant.GetFrameByName("base_link")
            _UP_Arm = self.plant.GetFrameByName("upper_arm_link")
            _Wrist = self.plant.GetFrameByName("wrist_3_link")
        elif 'roake' in self.arm_type:
            _Base = self.plant.GetFrameByName("xMatePro3_base")
            _UP_Arm = self.plant.GetFrameByName("xMatePro3_link3")
            _Wrist = self.plant.GetFrameByName("xMatePro3_link7")

        # 3. 通过正运动学计算机器人实际达到的旋转矩阵 R_rob
        actual_up_ori = self.plant.CalcRelativeRotationMatrix(self.context, _Base, _UP_Arm).matrix()
        actual_wrist_ori = self.plant.CalcRelativeRotationMatrix(self.context, _Base, _Wrist).matrix()

        # 4. 计算残差 (单位：弧度)。如果是 UR5，这个肩部残差将体现子流形投影损失
        shoulder_residual = self.CalcRotMetric(original_up_ori.matrix(), actual_up_ori)
        wrist_residual = self.CalcRotMetric(original_wrist_ori.matrix(), actual_wrist_ori)

        return shoulder_residual, wrist_residual

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

def decode_6d_to_rotation_matrix(rot_6d):
        """
        将 6D 旋转向量还原为 3x3 旋转矩阵 (基于 Gram-Schmidt 正交化)
        输入: rot_6d 形状为 (6,) 的 numpy 数组 [x1, y1, z1, x2, y2, z2]
        输出: 3x3 旋转矩阵
        """
        # 1. 提取前两列 (向量 a1 和 a2)
        a1 = rot_6d[:3]
        a2 = rot_6d[3:]
        
        # 2. 归一化第一列 -> b1
        b1 = a1 / np.linalg.norm(a1)
        
        # 3. 将第二列 a2 投影到 b1 的法平面上，并归一化 -> b2
        b2 = a2 - np.dot(b1, a2) * b1
        b2 = b2 / np.linalg.norm(b2)
        
        # 4. 叉乘得到第三列 -> b3 (保证右手定则)
        b3 = np.cross(b1, b2)
        
        # 5. 拼装成 3x3 矩阵
        return np.column_stack((b1, b2, b3))

def parse_arm_prediction(pred_arm_18d):
    """
    将 18 维预测输出拆解并还原为 3 个关节的旋转矩阵
    输入: pred_arm_18d 形状为 (18,)
    输出: list，包含 3 个 (3, 3) 旋转矩阵
    """
    rot_matrices = []
    for i in range(3): # 遍历 3 个关节
        # 每次切片提取 6 个数值
        rot_6d = pred_arm_18d[i*6 : (i+1)*6]
        rot_mat = decode_6d_to_rotation_matrix(rot_6d)
        rot_matrices.append(rot_mat)
    return rot_matrices

def load_config(config_path: str, model_name: str):
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"找不到配置文件: {config_path}")
        
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
        
    # 安全地获取路径，如果字典里没有这个 model_name，会返回 None
    mjcf_path = config.get('mujoco_models', {}).get(model_name)
    sdf_path = config.get('sdf_models', {}).get(model_name)
    
    if mjcf_path is None or sdf_path is None:
        raise ValueError(f"❌ 在 YAML 配置文件中未找到模型 '{model_name}' 的对应路径！请检查拼写。")
        
    return mjcf_path, sdf_path

def main():
    # 1. 配置参数
    bag_path = '/home/hanmg/Imitation_learning-Proj/raw_data/ros_topic/arm_action_test'  # 替换为你的 mcap 文件夹路径
    package_name = 'arm_hand_description'

    parser = argparse.ArgumentParser(description="离线提取机器人的子流形投影残差数据")
    # 添加必要的命令行参数
    parser.add_argument('-m', '--model_name', type=str, required=True, help="[必填] 机器人的型号名称，例如 'UR5', 'iiwa14' 或 'roake_7dof'")
    parser.add_argument('-b', '--bag_path', type=str, default='/home/hanmg/Imitation_learning-Proj/raw_data/ros_topic/arm_action_test', help="[选填] mcap 数据包所在的文件夹路径 ")
    parser.add_argument('-c', '--config', type=str, default='models.yaml', help="[选填] yaml 配置文件的路径")
    args = parser.parse_args()

    model_name = args.model_name
    bag_path = args.bag_path
    config_path = args.config
    print(f"🚀 开始离线解析 ROS 2 数据包: {bag_path}")
    print(f"🤖 目标验证模型: {model_name}")
    # 2. 初始化 IK 求解器
    try:
        mjcf_path, sdf_path = load_config(config_path, model_name)
        print(f"📄 成功加载 SDF 路径: {sdf_path}")
        # 如果你这里也需要启动 mujoco，可以直接使用 mjcf_path
        # print(f"📄 成功加载 MJCF 路径: {mjcf_path}")
    except Exception as e:
        print(e)
        return  # 读取配置失败则直接退出程序
    
    ik_solver = arm_ik(sdf_path, arm_type=model_name)
    q_last = ik_solver._q
    
    # 3. 准备数据容器
    shoulder_residuals = []
    wrist_residuals = []
    timestamps = []
    
    # 4. 离线极速读取 MCAP 数据
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == '/quat_data']
        if not connections:
            print("❌ 错误：在数据包中未找到 /quat_data 话题！")
            return
            
        print("✅ 找到话题，开始逐帧计算 IK 与投影残差...")
        
        # 遍历话题中的每一帧消息
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # 将底层二进制数据反序列化为 Python 对象 (对应 std_msgs/msg/Float32MultiArray)
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # --- 以下是你原本 quat_callback 中的解析逻辑 ---
            quats = np.array(msg.data)
            rots = Quatnumpy_to_Rotation(quats)
            
            # 提取原始的人类期望姿态 (基于你的转换逻辑)
            # 注意：Drake 的 RotationMatrix 对象调用 .matrix() 返回 np.ndarray
            des_wrist_ori = rots[0]
            des_elbow_ori = rots[1]
            des_up_ori = rots[2]
            
            # --- 求解 IK ---
            q_sol = ik_solver.ori_inv(
                up_ori=des_up_ori.matrix(), 
                elbow_ori=des_elbow_ori.matrix(), 
                wrist_ori=des_wrist_ori.matrix(), 
                q_last=q_last
            )
            
            # --- 计算残差并记录 ---
            if q_sol is not None:
                # 调用我们之前写好的专门计算残差的函数
                # 传入未被投影污染的 des_up_ori 和 des_wrist_ori (作为 Drake RotationMatrix 对象传入)
                res_shoulder, res_wrist = ik_solver.calc_and_get_residuals(
                    q_sol, des_up_ori, des_wrist_ori
                )
                
                shoulder_residuals.append(res_shoulder)
                wrist_residuals.append(res_wrist)
                timestamps.append(timestamp)
                
                # 更新 q_last 为下一帧做准备
                q_last = q_sol
            else:
                # 如果某帧无解，为了保证画图时时序对齐，填入 NaN
                print(f"⚠️ 警告: 时间戳 {timestamp} 处 IK 求解失败")
                shoulder_residuals.append(np.nan)
                wrist_residuals.append(np.nan)
                timestamps.append(timestamp)

    # 5. 数据落盘保存为 .npy
    output_dir = "residual_results"
    os.makedirs(output_dir, exist_ok=True)
    
    np.save(os.path.join(output_dir, f"{model_name}_shoulder_residuals.npy"), np.array(shoulder_residuals))
    np.save(os.path.join(output_dir, f"{model_name}_wrist_residuals.npy"), np.array(wrist_residuals))
    np.save(os.path.join(output_dir, "timestamps_ns.npy"), np.array(timestamps))
    
    print(f"🎉 处理完成！共提取 {len(shoulder_residuals)} 帧数据。")
    print(f"💾 结果已保存至 {output_dir}/ 目录下。")

if __name__ == '__main__':
    main()

