from omni.isaac.kit import SimulationApp
import numpy as np

#number of balls in the field
num_balls = 12
#shape of the ball with x, y and z
location_shape = (num_balls, 3)  
ball_locations = np.zeros(location_shape)

ball_paths = ["/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update", 
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_01", 
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_02", 
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_03", 
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_04", 
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_05", 
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_06",
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_07",
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_08",
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_09",
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_10",
              "/World/Field/_76_8354_000_Rev3_Portable_Field/NewBallFile/_76_8354_001_Rev5_12_8_Update_11"]




simulation_app = SimulationApp({"headless": False, "anti_aliasing": 0})
  
 
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd

my_world = World(stage_units_in_meters=1.0)
#self._my_world.scene.add_default_ground_plane()
field = add_reference_to_stage(usd_path="/home/opencav-krovi/VEX_AI/robot/FieldWithRobot.usd", prim_path="/World/Field")
vexbot_asset_path = "/home/opencav-krovi/VEX_AI/robot/vexbot.usd"
vexbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Body_Copy_1", 
        name="my_vexbot",
        wheel_dof_names=["dof_BL", "dof_BR"],
        create_robot=True,
        usd_path=vexbot_asset_path,
        position=np.array([0.5, -0.5, 0.3]),
        orientation=np.array([1.0, 150.0, 180.0, 180.0]),
    )
)
vexbot_controller = DifferentialController(name="simple_control", wheel_radius=3.96875, wheel_base=23.5)

while simulation_app.is_running():
    my_world.step(render=True)
    

    if my_world.is_playing():
        i = 0
        for ball in ball_paths:
            ball_locations[i] = Usd.get_prim_at_path(ball).get_world_pose()[0]
            i += 1
        

        print(ball_locations)


simulation_app.close()





 
   
 

