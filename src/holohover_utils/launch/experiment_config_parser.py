import os
from ament_index_python.packages import get_package_share_directory
import yaml

class ExperimentConfigParser:
    def __init__(self, config_file):
        self.experiment_conf = os.path.join(
            get_package_share_directory('holohover_utils'), 
            'config', 
            'experiments', 
            config_file
        )

        self.data = yaml.safe_load(open(self.experiment_conf, 'r'))
        hovercraft = self.data["hovercraft"]
        obstacles = self.data["obstacles"] if "obstacles" in self.data else []

        self.n_hovercraft = len(hovercraft)
        self.n_obstacles = len(obstacles)
        
        self.common_nodes_machine = self.data["experiment"]["machine"]
        
        rviz_props_file = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config',
            self.data["experiment"]["rviz_props_file"])
        
        self.hovercraft_machines = []
        self.hovercraft_names = []
        self.holohover_params = []
        self.hovercraft_ids = []
        self.initial_states = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}
        self.obst_initial_states = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}

        colors = []
        simulate = []

        for h in (hovercraft + obstacles):
            # print("Hovercraft: ", h)
            
            self.hovercraft_machines.append(h['machine'])
            self.hovercraft_names.append(h['name'])
            self.hovercraft_ids.append(int(h['id']))
            colors += h['color']
            simulate.append(h['simulate'])
            self.holohover_params.append(os.path.join(
                get_package_share_directory('holohover_utils'),
                'config',
                h['holohover_props']))

            initial_state = h['initial_state']
            for i, val in enumerate(initial_state):
                self.initial_states[list(self.initial_states.keys())[i]].append(float(val))

        for h in obstacles:
            initial_state = h['initial_state']
            for i, val in enumerate(initial_state):
                self.obst_initial_states[list(self.obst_initial_states.keys())[i]].append(float(val))

        self.sim_config = { 
                            "hovercraft_ids" :       self.hovercraft_ids, 
                            "hovercraft_names" :     self.hovercraft_names,
                            "initial_state_x":       self.initial_states['x'], 
                            "initial_state_y":       self.initial_states['y'], 
                            "initial_state_theta":   self.initial_states['theta'], 
                            "initial_state_vx":      self.initial_states['vx'], 
                            "initial_state_vy":      self.initial_states['vy'], 
                            "initial_state_w":       self.initial_states['w'],
                            "holohover_props_files": self.holohover_params,
                            "rviz_props_file" :      rviz_props_file,
                            "color" :                colors,
                            "simulated" :             simulate
                          }

    def getSimConfig(self):
        return self.sim_config
    
    def getDMPCdata(self): # ToDo evaluate to move this to dmpc config
        opt_alg = self.data["experiment"]["opt_alg"] # admm or dsqp
        file_name_xd_trajectory = self.data["experiment"]["file_name_xd_trajectory"]
        file_name_ud_trajectory = self.data["experiment"]["file_name_ud_trajectory"]
        dmpc_config_folder = self.data["experiment"]["dmpc_config_folder"]

        return opt_alg,file_name_xd_trajectory,file_name_ud_trajectory, dmpc_config_folder
    
    def getCommonNodesMachine(self):
        return self.common_nodes_machine
        
    def getHovercraft(self):
        return self.hovercraft_machines[:self.n_hovercraft], self.hovercraft_names[:self.n_hovercraft], self.holohover_params[:self.n_hovercraft], self.initial_states 

    def getObstacles(self):
        return self.hovercraft_machines[self.n_hovercraft:], self.hovercraft_names[self.n_hovercraft:], self.holohover_params[self.n_hovercraft:], self.obst_initial_states 

    def getExperimentNameDesc(self):
        return self.data["experiment"]["name"], self.data["experiment"]["description"]
    
    def getHovercraftNamesIds(self):
        return self.hovercraft_names, self.hovercraft_ids
    
