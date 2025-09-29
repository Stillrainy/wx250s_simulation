import numpy as np
from urchin import URDF
from typing import List, Union
import logging
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
URDF_PATH = BASE_DIR / 'assets' / 'wx250s.urdf'


class wx250s:
    """A robot simulation class for the WX250s robotic arm.
    
    This class provides an interface to control and simulate the WX250s robotic arm
    using URDF-based forward kinematics. It supports joint position control,
    end-effector pose calculation, and visualization.
    
    Attributes:
        DoF (int): Degrees of freedom for the robot (6)
        joint_limits (np.ndarray): Joint position limits for each actuated joint
        tool_T_tip_offset (np.ndarray): Transformation offset from tool to tip
    """
    
    def __init__(self, urdf_path: str = str(URDF_PATH)):
        """Initialize the WX250s robot simulation.
        
        Args:
            urdf_path (str): Path to the URDF file describing the robot.
                           Defaults to 'assets/wx250s.urdf'.
        """
        self._robot = URDF.load(urdf_path)
        self.tool_T_tip_offset = np.eye(4)

        self.DoF = 6
        self.joint_limits = self._robot.joint_limits[:self.DoF]
        self._DoF = len(self._robot.actuated_joints)
        self._joint_positions = np.zeros(self._DoF)
        # Set the zero position offset for joint 2 and 3
        self._zero_offset = np.arctan(50/250)
        self._apply_offset()

    def set_joint_positions(
            self,
            joint_positions: Union[List[float], np.ndarray]
    ) -> bool:
        """Set the joint positions of the robot.
        
        Args:
            joint_positions: Target joint positions for the robot's actuated joints.
                           Must contain exactly 6 values (matching DoF).
        
        Returns:
            bool: True if joint positions were successfully set, False if validation failed.
        """
        if self._check_inputs(joint_positions):
            self._joint_positions[:self.DoF] = joint_positions
            self._apply_offset()
            return True
        else:
            return False

    def get_joint_positions(self) -> List[float]:
        """Get the current joint positions of the robot.
        
        Returns:
            List[float]: Current joint positions for all DoF joints.
        """
        joint_positions = self._joint_positions[:self.DoF].copy()
        joint_positions[1] += self._zero_offset
        joint_positions[2] -= self._zero_offset
        return joint_positions.tolist()

    def get_ee_pose(self) -> np.ndarray:
        """Calculate the end-effector pose using forward kinematics.
        
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix representing
                       the pose of the end-effector (gripper link) in the base frame.
        """
        return self._robot.link_fk(
            self._joint_positions,
            use_names=True)['wx250s/ee_gripper_link']

    def go_to_home_pose(self) -> None:
        """Move the robot to its home pose.
        
        Sets all joint positions to zero, which corresponds to the robot's
        default/home configuration.
        """
        self.set_joint_positions(np.zeros(self.DoF))

    def show(self) -> None:
        """Display a 3D visualization of the robot in its current configuration.
        
        Opens an interactive 3D viewer showing the robot at its current joint positions.
        Requires a display environment to render the visualization.
        """
        self._robot.show(self._joint_positions)

    def _check_inputs(
            self,
            joint_positions: Union[List[float], np.ndarray]
    ) -> bool:
        """Validate joint position inputs against robot constraints.
        
        Args:
            joint_positions: Joint positions to validate.
        
        Returns:
            bool: True if all joint positions are valid, False otherwise.
        """
        if len(joint_positions) != self.DoF:
            logging.warning(
                f"Input joint positions length {len(joint_positions)} does not match DoF {self.DoF}.")
            return False

        joint_positions = np.array(joint_positions)
        lower_limits = self.joint_limits[:, 0]
        upper_limits = self.joint_limits[:, 1]

        below_limits = np.where(joint_positions < lower_limits)[0]
        if below_limits.size > 0:
            for idx in below_limits:
                logging.warning(
                    f"Joint {idx} position {joint_positions[idx]:.2f} is below lower limit {lower_limits[idx]:.2f}.")
            return False

        above_limits = np.where(joint_positions > upper_limits)[0]
        if above_limits.size > 0:
            for idx in above_limits:
                logging.warning(
                    f"Joint {idx} position {joint_positions[idx]:.2f} is above upper limit {upper_limits[idx]:.2f}.")
            return False
        return True

    def _apply_offset(self) -> np.ndarray:
        """Apply zero position offset to specific joints.
        """
        self._joint_positions[1] -= self._zero_offset
        self._joint_positions[2] += self._zero_offset
