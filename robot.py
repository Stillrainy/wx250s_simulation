import numpy as np
from urchin import URDF
from typing import List, Union
import logging


class wx250s:
    def __init__(self, urdf_path: str = 'assets/wx250s.urdf'):
        self._robot = URDF.load(urdf_path)
        self.tool_T_tip_offset = np.eye(4)

        self.DoF = 6
        self.joint_limits = self._robot.joint_limits[:self.DoF]
        self._DoF = len(self._robot.actuated_joints)
        self._joint_positions = np.zeros(self._DoF)

    def set_joint_positions(
            self,
            joint_positions: Union[List[float], np.ndarray]
    ) -> bool:
        if self._check_inputs(joint_positions):
            self._joint_positions[:self.DoF] = joint_positions
            return True
        else:
            return False

    def get_joint_positions(self) -> List[float]:
        return self._joint_positions[:self.DoF].tolist()

    def get_ee_pose(self) -> np.ndarray:
        return self._robot.link_fk(
            self._joint_positions,
            use_names=True)['wx250s/ee_gripper_link']

    def go_to_home_pose(self) -> None:
        self.set_joint_positions(np.zeros(self.DoF))

    def show(self) -> None:
        self._robot.show(self._joint_positions)

    def animate(self, cfgs) -> None:
        self._robot.animate(cfgs)
        pass

    def _check_inputs(
            self,
            joint_positions: Union[List[float], np.ndarray]
    ) -> bool:
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
