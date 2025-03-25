import mujoco as mj
import yaml
from launch_ros.substitutions import FindPackageShare
import os

class mj_armHandSys():
    def __init__(self, model_name,package_name):

        config_path = os.path.join(FindPackageShare(package = 'arm_tele_mujo').find('arm_tele_mujo'),'config/models.yaml')
        
        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)  
            self.mujoco_models = config_data['mujoco_models']
            self.sdf_models = config_data['sdf_models']

        ## Load MuJoCo model
        assert model_name in self.mujoco_models, f"Model {model_name} not found in models.yaml"
        model_path = self.mujoco_models[model_name]
        self.pkg_share = FindPackageShare(package = package_name).find(package_name)
        self.mjcf_model_path = os.path.join(self.pkg_share,f'{model_path}')

        self.model = mj.MjModel.from_xml_path(self.mjcf_model_path)
        self.data = mj.MjData(self.model)

        ## Load SDF model
        assert model_name in self.sdf_models, f"Model {model_name} not found in models.yaml"
        self.sdf_model = self.sdf_models[model_name]



