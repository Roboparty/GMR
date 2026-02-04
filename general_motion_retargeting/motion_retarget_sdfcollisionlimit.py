
import mink
import mujoco as mj
import numpy as np
import json
from scipy.spatial.transform import Rotation as R
from .params import ROBOT_XML_DICT, IK_CONFIG_DICT
from rich import print

class SDFCollisionLimit:
    """Collision avoidance using SDF (Signed Distance Field) with short-term smoothing."""

    def __init__(
        self,
        model,
        key_bodies=None,
        gain=0.5,
        min_distance=0.02,
        sdf_function=None,
    ):
        self.model = model
        self.gain = gain
        self.min_distance = min_distance
        self.sdf_function = sdf_function or self._default_sdf
        self.key_bodies = key_bodies or ["ankle"]

        # 自动选择每个 key body 的第一个 collision geom
        self.body_to_geom = self._select_first_collision_geom()

    def _select_first_collision_geom(self):
        body_to_geom = {}
        for geom_id in range(self.model.ngeom):
            body_id = self.model.geom_bodyid[geom_id]
            body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, body_id)
            if body_name is not None:
                body_name = body_name.lower()
            if any(k in body_name for k in self.key_bodies):
                if body_name not in body_to_geom:
                    body_to_geom[body_name] = geom_id
        return body_to_geom

    def _default_sdf(self, data: mj.MjData, geom_id: int):
        fromto = np.empty(6)
        min_dist = float('inf')
        min_grad = np.zeros(3)
        for other_body, other_geom in self.body_to_geom.items():
            if other_geom == geom_id:
                continue
            dist = mj.mj_geomDistance(self.model, data, geom_id, other_geom, 0.1, fromto)
            if dist < min_dist:
                min_dist = dist
                grad = fromto[3:] - fromto[:3]
                mj.mju_normalize3(grad)
                min_grad = grad
        return min_dist, min_grad

    def compute_tasks(self, configuration: mink.Configuration, history_grads=None, smoothing_factor=0.5):
        """Returns a list of FrameTasks for SDF collision avoidance with gradient smoothing"""
        tasks = []
        new_history_grads = {}

        for body_name, geom_id in self.body_to_geom.items():
            body_id = mj.mj_name2id(configuration.model, mj.mjtObj.mjOBJ_BODY, body_name)
            pos = configuration.data.xpos[body_id]

            dist, grad = self.sdf_function(configuration.data, geom_id)

            # 使用历史梯度做指数平滑
            if history_grads is not None and body_name in history_grads:
                grad = smoothing_factor * grad + (1 - smoothing_factor) * history_grads[body_name]

            new_history_grads[body_name] = grad.copy()

            if dist < self.min_distance:
                direction = grad / (np.linalg.norm(grad) + 1e-8)
                scale = min(1.0, (self.min_distance - dist) / self.min_distance)
                target_pos = pos + direction * (self.min_distance - dist) * self.gain * scale

                task = mink.FrameTask(
                    frame_name=body_name,
                    frame_type="body",
                    position_cost=self.gain,
                    orientation_cost=0.0,
                    lm_damping=1,
                )
                task.set_target(
                    mink.SE3.from_rotation_and_translation(
                        mink.SO3.from_matrix(np.eye(3)),
                        target_pos
                    )
                )
                tasks.append(task)

        return tasks, new_history_grads



class GeneralMotionRetargeting:
    """General Motion Retargeting (GMR) with Collision Avoidance and Motion Regularization.
    """
    def __init__(
        self,
        src_human: str,
        tgt_robot: str,
        actual_human_height: float = None,
        solver: str="daqp", # change from "quadprog" to "daqp".
        damping: float=5e-1, # change from 1e-1 to 1e-2.
        verbose: bool=True,
        use_velocity_limit: bool=False,
    ) -> None:

        # load the robot model
        self.xml_file = str(ROBOT_XML_DICT[tgt_robot])
        if verbose:
            print("Use robot model: ", self.xml_file)
        self.model = mj.MjModel.from_xml_path(self.xml_file)
        
        # Print DoF names in order
        print("[GMR] Robot Degrees of Freedom (DoF) names and their order:")
        self.robot_dof_names = {}
        for i in range(self.model.nv):  # 'nv' is the number of DoFs
            dof_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, self.model.dof_jntid[i])
            self.robot_dof_names[dof_name] = i
            if verbose:
                print(f"DoF {i}: {dof_name}")
            
            
        print("[GMR] Robot Body names and their IDs:")
        self.robot_body_names = {}
        for i in range(self.model.nbody):  # 'nbody' is the number of bodies
            body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
            self.robot_body_names[body_name] = i
            if verbose:
                print(f"Body ID {i}: {body_name}")
        
        print("[GMR] Robot Motor (Actuator) names and their IDs:")
        self.robot_motor_names = {}
        for i in range(self.model.nu):  # 'nu' is the number of actuators (motors)
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[motor_name] = i
            if verbose:
                print(f"Motor ID {i}: {motor_name}")

        # Load the IK config
        with open(IK_CONFIG_DICT[src_human][tgt_robot]) as f:
            ik_config = json.load(f)
        if verbose:
            print("Use IK config: ", IK_CONFIG_DICT[src_human][tgt_robot])
        
        # compute the scale ratio based on given human height and the assumption in the IK config
        if actual_human_height is not None:
            ratio = actual_human_height / ik_config["human_height_assumption"]
        else:
            ratio = 1.0
            
        # adjust the human scale table
        for key in ik_config["human_scale_table"].keys():
            ik_config["human_scale_table"][key] = ik_config["human_scale_table"][key] * ratio
    

        # used for retargeting
        self.ik_match_table1 = ik_config["ik_match_table1"]
        self.ik_match_table2 = ik_config["ik_match_table2"]
        self.human_root_name = ik_config["human_root_name"]
        self.robot_root_name = ik_config["robot_root_name"]
        self.use_ik_match_table1 = ik_config["use_ik_match_table1"]
        self.use_ik_match_table2 = ik_config["use_ik_match_table2"]
        self.human_scale_table = ik_config["human_scale_table"]
        self.ground = ik_config["ground_height"] * np.array([0, 0, 1])

        self.max_iter = 10

        self.solver = solver
        self.damping = damping

        self.human_body_to_task1 = {}
        self.human_body_to_task2 = {}
        self.pos_offsets1 = {}
        self.rot_offsets1 = {}
        self.pos_offsets2 = {}
        self.rot_offsets2 = {}

        self.task_errors1 = {}
        self.task_errors2 = {}

        self.ik_limits = [mink.ConfigurationLimit(self.model)]
        if use_velocity_limit:
            VELOCITY_LIMITS = {k: 3*np.pi for k in self.robot_motor_names.keys()}
            self.ik_limits.append(mink.VelocityLimit(self.model, VELOCITY_LIMITS))
        
        # ============= 新增：碰撞和正则化配置 =============
        self._sdf_history_grads = None
        self.collision_enabled = False
        self.sdf_limit = None
        self.collision_gain = 10
        self.collision_safety_margin = 0.05
        # self.collision_body_names = ['knee', 'ankle', 'torso', 'elbow', 'hand']
        self.collision_body_names = ['ankle_roll', 'knee']
        
        self.regularity_enabled = True
        self.acc_weight = 0.01
        self.jerk_weight = 0.001
        self.prev_vel = None
        self.prev_acc = None
        # ================================================
            
        self.setup_retarget_configuration()
        self.sdf_limit = SDFCollisionLimit(
            model=self.model,
            key_bodies=self.collision_body_names,
            gain=self.collision_gain,
            min_distance=self.collision_safety_margin,
        )
        self.set_regularity(enabled=self.regularity_enabled,
                            acc_weight=self.acc_weight,
                            jerk_weight=self.jerk_weight)
        self.ground_offset = 0.0

    def setup_retarget_configuration(self):
        self.configuration = mink.Configuration(self.model)
    
        self.tasks1 = []
        self.tasks2 = []
        
        for frame_name, entry in self.ik_match_table1.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task1[body_name] = task
                self.pos_offsets1[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets1[body_name] = R.from_quat(
                    rot_offset, scalar_first=True
                )
                self.tasks1.append(task)
                self.task_errors1[task] = []
        
        for frame_name, entry in self.ik_match_table2.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task2[body_name] = task
                self.pos_offsets2[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets2[body_name] = R.from_quat(
                    rot_offset, scalar_first=True
                )
                self.tasks2.append(task)
                self.task_errors2[task] = []

    # ============= 新增:运动正则化设置 =============
    def set_regularity(
        self,
        enabled: bool = True,
        acc_weight: float = 0.01,
        jerk_weight: float = 0.001
    ):
        """设置运动正则化（加速度和加加速度）
        
        Args:
            enabled: 是否启用正则化
            acc_weight: 加速度正则化权重 (建议范围: 0.001-0.1)
            jerk_weight: 加加速度正则化权重 (建议范围: 0.0001-0.01)
        """
        self.regularity_enabled = enabled
        self.acc_weight = acc_weight
        self.jerk_weight = jerk_weight
        
        if enabled:
            print(f"[GMR] Motion regularization enabled (acc={acc_weight}, jerk={jerk_weight})")
        else:
            print("[GMR] Motion regularization disabled")
            self.reset_regularity()
    
    def reset_regularity(self):
        """重置正则化的历史状态（用于新序列开始）"""
        self.prev_vel = None
        self.prev_acc = None
        if self.regularity_enabled:
            print("[GMR] Regularization history reset")
    
    def _get_tasks_with_constraints(self, base_tasks, vel, dt):
        """获取带约束的任务列表"""
        extended_tasks = base_tasks.copy()
        
        # 运动正则化仍然作为任务添加
        if self.regularity_enabled and vel is not None and self.prev_vel is not None:
            current_acc = (vel - self.prev_vel) / dt
            if self.acc_weight > 0:
                acc_task = mink.PostureTask(model=self.model, cost=self.acc_weight)
                acc_task.set_target_from_configuration(self.configuration)
                extended_tasks.append(acc_task)
            if self.jerk_weight > 0 and self.prev_acc is not None:
                jerk_task = mink.PostureTask(model=self.model, cost=self.jerk_weight)
                jerk_task.set_target_from_configuration(self.configuration)
                extended_tasks.append(jerk_task)
            self.prev_acc = current_acc.copy() if self.prev_acc is None else self.prev_acc

        return extended_tasks
    
    # ============================================

    def update_targets(self, human_data, offset_to_ground=False):
        # scale human data in local frame
        human_data = self.to_numpy(human_data)
        human_data = self.scale_human_data(human_data, self.human_root_name, self.human_scale_table)
        human_data = self.offset_human_data(human_data, self.pos_offsets1, self.rot_offsets1)
        human_data = self.apply_ground_offset(human_data)
        if offset_to_ground:
            human_data = self.offset_human_data_to_ground(human_data)
        self.scaled_human_data = human_data

        if self.use_ik_match_table1:
            for body_name in self.human_body_to_task1.keys():
                task = self.human_body_to_task1[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
        
        if self.use_ik_match_table2:
            for body_name in self.human_body_to_task2.keys():
                task = self.human_body_to_task2[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
            
            
    def retarget(self, human_data, offset_to_ground=False):
        # 更新任务目标
        self.update_targets(human_data, offset_to_ground)
        
        dt = self.configuration.model.opt.timestep

        # ------------------ 准备 IK 限制 ------------------
        limits = self.ik_limits.copy()
        history_grads = getattr(self, "_sdf_history_grads", None)

        # ------------------ IK Match Table 1 ------------------
        if self.use_ik_match_table1:
            curr_error = self.error1()
            num_iter = 0

            while num_iter < self.max_iter:
                # 获取基础任务 + 正则化
                tasks = self._get_tasks_with_constraints(
                    self.tasks1,
                    None if self.prev_vel is None else self.prev_vel,
                    dt
                )

                # 添加 SDF 碰撞任务
                if self.collision_enabled and self.sdf_limit is not None:
                    sdf_tasks, history_grads = self.sdf_limit.compute_tasks(self.configuration, history_grads)
                    tasks += sdf_tasks

                # IK 求解
                vel1 = mink.solve_ik(
                    self.configuration,
                    tasks,
                    dt,
                    self.solver,
                    self.damping,
                    limits=limits
                )

                # 更新历史速度
                if self.regularity_enabled:
                    self.prev_vel = vel1.copy()

                # 积分更新配置
                self.configuration.integrate_inplace(vel1, dt)

                # 误差更新
                next_error = self.error1()
                if curr_error - next_error < 0.001:
                    break
                curr_error = next_error
                num_iter += 1

        # ------------------ IK Match Table 2 ------------------
        if self.use_ik_match_table2:
            curr_error = self.error2()
            num_iter = 0

            while num_iter < self.max_iter:
                tasks = self._get_tasks_with_constraints(
                    self.tasks2,
                    None if self.prev_vel is None else self.prev_vel,
                    dt
                )

                # 添加 SDF 碰撞任务
                if self.collision_enabled and self.sdf_limit is not None:
                    sdf_tasks, history_grads = self.sdf_limit.compute_tasks(self.configuration, history_grads)
                    tasks += sdf_tasks

                vel2 = mink.solve_ik(
                    self.configuration,
                    tasks,
                    dt,
                    self.solver,
                    self.damping,
                    limits=limits
                )

                if self.regularity_enabled:
                    self.prev_vel = vel2.copy()

                self.configuration.integrate_inplace(vel2, dt)

                next_error = self.error2()
                if curr_error - next_error < 0.001:
                    break
                curr_error = next_error
                num_iter += 1
        self._sdf_history_grads = history_grads
        return self.configuration.data.qpos.copy()

    def error1(self):
        return np.linalg.norm(
            np.concatenate(
                [task.compute_error(self.configuration) for task in self.tasks1]
            )
        )
    
    def error2(self):
        return np.linalg.norm(
            np.concatenate(
                [task.compute_error(self.configuration) for task in self.tasks2]
            )
        )


    def to_numpy(self, human_data):
        for body_name in human_data.keys():
            human_data[body_name] = [np.asarray(human_data[body_name][0]), np.asarray(human_data[body_name][1])]
        return human_data


    def scale_human_data(self, human_data, human_root_name, human_scale_table):
        
        human_data_local = {}
        root_pos, root_quat = human_data[human_root_name]
        
        # scale root
        scaled_root_pos = human_scale_table[human_root_name] * root_pos
        
        # scale other body parts in local frame
        for body_name in human_data.keys():
            if body_name not in human_scale_table:
                continue
            if body_name == human_root_name:
                continue
            else:
                # transform to local frame (only position)
                human_data_local[body_name] = (human_data[body_name][0] - root_pos) * human_scale_table[body_name]
            
        # transform the human data back to the global frame
        human_data_global = {human_root_name: (scaled_root_pos, root_quat)}
        for body_name in human_data_local.keys():
            human_data_global[body_name] = (human_data_local[body_name] + scaled_root_pos, human_data[body_name][1])

        return human_data_global
    
    def offset_human_data(self, human_data, pos_offsets, rot_offsets):
        """the pos offsets are applied in the local frame"""
        offset_human_data = {}
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            offset_human_data[body_name] = [pos, quat]
            # apply rotation offset first
            updated_quat = (R.from_quat(quat, scalar_first=True) * rot_offsets[body_name]).as_quat(scalar_first=True)
            offset_human_data[body_name][1] = updated_quat
            
            local_offset = pos_offsets[body_name]
            # compute the global position offset using the updated rotation
            global_pos_offset = R.from_quat(updated_quat, scalar_first=True).apply(local_offset)
            
            offset_human_data[body_name][0] = pos + global_pos_offset
           
        return offset_human_data
            
    def offset_human_data_to_ground(self, human_data):
        """find the lowest point of the human data and offset the human data to the ground"""
        offset_human_data = {}
        ground_offset = 0.02
        lowest_pos = np.inf

        for body_name in human_data.keys():
            # only consider the foot/Foot
            if "Foot" not in body_name and "foot" not in body_name:
                continue
            pos, quat = human_data[body_name]
            if pos[2] < lowest_pos:
                lowest_pos = pos[2]
                lowest_body_name = body_name
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            offset_human_data[body_name] = [pos, quat]
            offset_human_data[body_name][0] = pos - np.array([0, 0, lowest_pos]) + np.array([0, 0, ground_offset])
        return offset_human_data

    def set_ground_offset(self, ground_offset):
        self.ground_offset = ground_offset

    def apply_ground_offset(self, human_data):
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            human_data[body_name][0] = pos - np.array([0, 0, self.ground_offset])
        return human_data
