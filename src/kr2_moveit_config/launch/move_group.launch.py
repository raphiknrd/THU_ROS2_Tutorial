from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kr2_robot_a1018", package_name="kr2_moveit_config")
        .to_moveit_configs()
    )
    
    # Wir setzen den Parameter direkt in der generierten Konfiguration
    # Das ist die "saubere" Art, die vom Builder unterstützt wird
    moveit_config.robot_description_kinematics['use_sim_time'] = True
    
    return generate_move_group_launch(moveit_config)