import math

from isaaclab.utils import configclass

from bipedal_locomotion.assets.config.LW import LW_CFG
from bipedal_locomotion.tasks.locomotion.cfg.LW_Leg.limx_base_env_cfg import LWLegEnvCfg
from bipedal_locomotion.tasks.locomotion.cfg.LW_Leg.terrains_cfg import (
    BLIND_ROUGH_TERRAINS_CFG,
    BLIND_ROUGH_TERRAINS_PLAY_CFG,
    STAIRS_TERRAINS_CFG,
    STAIRS_TERRAINS_PLAY_CFG,
)

from isaaclab.sensors import RayCasterCfg, patterns
from bipedal_locomotion.tasks.locomotion import mdp
from isaaclab.utils.noise import AdditiveGaussianNoiseCfg as GaussianNoise
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg


######################
# LW Leg Base Environment
######################


@configclass
class LWLegBaseEnvCfg(LWLegEnvCfg):

    base_link_name = "base_link"
    foot_link_name = ".*_foot_link"
    # fmt: off
    joint_names_without_wheels = [
        "right_hip_joint",
        "left_hip_joint",
        "right_thigh_joint",
        "left_thigh_joint",
        "right_shank_joint",
        "left_shank_joint",
        "right_foot_joint",
        "left_foot_joint",
    ]
    wheel_joint_names = [
        "right_wheel_joint",
        "left_wheel_joint",
    ]
    joint_names = joint_names_without_wheels + wheel_joint_names

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = LW_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # observations
        self.observations.policy.joint_pos.func = mdp.joint_pos_rel_without_wheel
        self.observations.policy.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.wheel_joint_names
        )
        self.observations.obsHistory.joint_pos.func = mdp.joint_pos_rel_without_wheel
        self.observations.obsHistory.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.wheel_joint_names
        )
        self.observations.critic.joint_pos.func = mdp.joint_pos_rel_without_wheel
        self.observations.critic.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.wheel_joint_names
        )

        self.observations.policy.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.policy.joint_vel.params["asset_cfg"].joint_names = self.joint_names
        self.observations.obsHistory.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.obsHistory.joint_vel.params["asset_cfg"].joint_names = self.joint_names
        self.observations.critic.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.critic.joint_vel.params["asset_cfg"].joint_names = self.joint_names

        # actions
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_vel.scale = 1.0
        self.actions.joint_pos.joint_names = self.joint_names_without_wheels
        self.actions.joint_vel.joint_names = self.wheel_joint_names

        # events
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 3.5)
        self.events.add_link_mass.params["mass_distribution_params"] = (0.7, 1.3)
        self.events.radomize_rigid_body_mass_inertia.params["mass_inertia_distribution_params"] = (0.7, 1.3)
        self.events.robot_center_of_mass.params["com_distribution_params"] = ((-0.075, 0.025), (-0.075, 0.075), (-0.075, 0.075))
        
        # update viewport camera
        self.viewer.origin_type = "env"


@configclass
class LWLegBaseEnvCfg_PLAY(LWLegBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 32

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.push_robot = None
        # remove random base mass addition event
        self.events.add_base_mass = None


############################
# Solefoot Blind Flat Environment
############################


@configclass
class LWLegBlindFlatEnvCfg(LWLegBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        # self.observations.policy.heights = None
        self.observations.critic.heights = None

        self.curriculum.terrain_levels = None


@configclass
class LWLegBlindFlatEnvCfg_PLAY(LWLegBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = None
        # self.observations.policy.heights = None
        self.observations.critic.heights = None

        self.curriculum.terrain_levels = None


#############################
# LW Leg Blind Rough Environment
#############################


@configclass
class LWLegBlindRoughEnvCfg(LWLegBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = None
        # self.observations.policy.heights = None
        self.observations.critic.heights = None

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = BLIND_ROUGH_TERRAINS_CFG


@configclass
class LWLegBlindRoughEnvCfg_PLAY(LWLegBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = None
        # self.observations.policy.heights = None
        self.observations.critic.heights = None
        
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = BLIND_ROUGH_TERRAINS_PLAY_CFG
        

##############################
# LW Leg Blind Stairs Environment
##############################

@configclass
class LWLegBlindStairEnvCfg(LWLegBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = None
        # self.observations.policy.heights = None
        self.observations.critic.heights = None

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-math.pi / 6, math.pi / 6)

        self.rewards.rew_lin_vel_xy.weight = 2.0
        self.rewards.rew_ang_vel_z.weight = 1.5
        self.rewards.pen_lin_vel_z.weight = -1.0
        self.rewards.pen_ang_vel_xy.weight = -0.05
        self.rewards.pen_action_rate.weight = -0.01
        self.rewards.pen_flat_orientation.weight = -2.5
        self.rewards.pen_undesired_contacts.weight = -1.0

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_CFG


@configclass
class LWLegBlindStairEnvCfg_PLAY(LWLegBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = None
        # self.observations.policy.heights = None
        self.observations.critic.heights = None

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.0, 0.0)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (-0.0, 0.0)

        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_PLAY_CFG.replace(difficulty_range=(0.5, 0.5))
        
        
#############################
# LW Leg Flat Environment
#############################

@configclass
class LWLegFlatEnvCfg(LWLegBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.5, 0.5]), #TODO: adjust size to fit real robot
            debug_vis=False,
            mesh_prim_paths=["/World/ground"],
        )
        self.observations.policy.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
                    noise=GaussianNoise(mean=0.0, std=0.01),
        )
        self.observations.critic.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
        )
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        self.curriculum.terrain_levels = None

@configclass
class LWLegFlatEnvCfg_PLAY(LWLegBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.5, 0.5]), #TODO: adjust size to fit real robot
            debug_vis=False,
            mesh_prim_paths=["/World/ground"],
        )
        self.observations.policy.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
        )
        self.observations.critic.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
        )
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        self.curriculum.terrain_levels = None
        
        
#############################
# LW Leg Rough Environment
#############################

@configclass
class LWLegRoughEnvCfg(LWLegBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.5, 0.5]), #TODO: adjust size to fit real robot
            debug_vis=False,
            mesh_prim_paths=["/World/ground"],
        )
        self.observations.policy.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
                    noise=GaussianNoise(mean=0.0, std=0.01),
        )
        self.observations.critic.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
        )
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = BLIND_ROUGH_TERRAINS_CFG

        # update viewport camera
        self.viewer.origin_type = "env"


@configclass
class LWLegRoughEnvCfg_PLAY(LWLegBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.5, 0.5]), #TODO: adjust size to fit real robot
            debug_vis=False,
            mesh_prim_paths=["/World/ground"],
        )
        self.observations.policy.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
        )
        self.observations.critic.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
        )
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = BLIND_ROUGH_TERRAINS_PLAY_CFG
        
##############################
# LW Leg Blind Stairs Environment
##############################


@configclass
class LWLegStairEnvCfg(LWLegBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.5, 0.5]), #TODO: adjust size to fit real robot
            debug_vis=False,
            mesh_prim_paths=["/World/ground"],
        )
        self.observations.policy.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
                    noise=GaussianNoise(mean=0.0, std=0.01),
                    clip = (0.0, 10.0),
        )
        self.observations.critic.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip = (0.0, 10.0),
        )
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-math.pi / 6, math.pi / 6)

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_CFG


@configclass
class LWLegStairEnvCfg_PLAY(LWLegBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.5, 0.5]), #TODO: adjust size to fit real robot
            debug_vis=True,
            mesh_prim_paths=["/World/ground"],
        )
        self.observations.policy.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip = (0.0, 10.0),
        )
        self.observations.critic.heights = ObsTerm(func=mdp.height_scan,
            params = {"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip = (0.0, 10.0),
        )
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.0, 0.0)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (-0.0, 0.0)

        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_PLAY_CFG.replace(difficulty_range=(0.5, 0.5))
        


