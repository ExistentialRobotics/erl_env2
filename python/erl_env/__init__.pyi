from typing import overload
from typing import List
from typing import Dict
from typing import Callable
import numpy as np
import numpy.typing as npt
from enum import IntEnum
from erl_common.storage import GridMapUnsigned2D
from erl_common.storage import GridMapInfo2D
from erl_common.storage import GridMapInfo3D
from erl_common.yaml import YamlableBase

__all__ = [
    "EnvironmentState",
    "Successor",
    "CostBase",
    "EuclideanDistanceCost",
    "ManhattanDistanceCost",
    "EnvironmentBase",
    "Environment2D",
    "EnvironmentSe2",
    "DifferentialDriveControl",
    "DdcMotionPrimitive",
    "load_ddc_motion_primitives_from_yaml",
    "EnvironmentAnchor",
    "EnvironmentGridAnchor2D",
    "EnvironmentGridAnchor3D",
]

class EnvironmentState:
    metric: npt.NDArray[np.float64]
    grid: npt.NDArray[np.int32]

class Successor:
    env_state: EnvironmentState
    cost: float
    action_coords: List[int]

class CostBase:
    def __call__(self: CostBase, state1: EnvironmentState, state2: EnvironmentState) -> float: ...

class EuclideanDistanceCost(CostBase):
    def __init__(self: EuclideanDistanceCost) -> None: ...

class ManhattanDistanceCost(CostBase):
    def __init__(self: ManhattanDistanceCost) -> None: ...

class EnvironmentBase:
    def __init__(self: EnvironmentBase, distance_cost_func: CostBase, time_step: float) -> None: ...
    @property
    def state_space_size(self: EnvironmentBase) -> int: ...
    @property
    def action_space_size(self: EnvironmentBase) -> int: ...
    def forward_action(
        self: EnvironmentBase, env_state: EnvironmentState, action_coords: int
    ) -> List[EnvironmentState]: ...
    @property
    def distance_cost_func(self: EnvironmentBase) -> CostBase: ...
    def get_successors(self: EnvironmentBase, env_state: EnvironmentState) -> List[Successor]: ...
    def in_state_space(self: EnvironmentBase, env_state: EnvironmentState) -> bool: ...
    def state_hashing(self: EnvironmentBase, env_state: EnvironmentState) -> int: ...
    def metric_to_grid(self: EnvironmentBase, metric_state: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: ...
    def grid_to_metric(self: EnvironmentBase, grid_state: npt.NDArray[np.int32]) -> npt.NDArray[np.int32]: ...
    def show_paths(self: EnvironmentBase, paths: Dict[int, npt.NDArray[np.float]]) -> None: ...

class Environment2D(EnvironmentBase):
    class Action(IntEnum):
        kForward = (0,)
        kBack = (1,)
        kRight = (2,)
        kLeft = (3,)
        kForwardRight = (4,)
        kForwardLeft = (5,)
        kBackRight = (6,)
        kBackLeft = 7

    class Setting(YamlableBase):
        allow_diagonal: bool
        step_size: int
        down_sampled: bool
        obstacle_threshold: float
        add_map_cost: bool
        map_cost_factor: float
        shape: npt.NDArray[np.float64]
    def __init__(
        self: Environment2D,
        grid_map: GridMapUnsigned2D,
        setting: Setting = None,
        distance_cost_func: CostBase = None,
    ) -> None: ...
    @staticmethod
    def get_action_from_name(action_name: str) -> Action: ...

class DifferentialDriveControl:
    @overload
    def __init__(self: DifferentialDriveControl) -> None: ...
    @overload
    def __init__(self: DifferentialDriveControl, linear_velocity: float, angular_velocity: float) -> None: ...

    linear_v: float
    angular_v: float

class DdcMotionPrimitive:
    @overload
    def __init__(self: DdcMotionPrimitive) -> None: ...
    @overload
    def __init__(
        self: DdcMotionPrimitive,
        controls: List[DifferentialDriveControl],
        durations: List[float],
        costs: List[float],
    ) -> None: ...

    controls: List[DifferentialDriveControl]
    durations: List[float]
    costs: List[float]

    def compute_trajectories(
        self: DdcMotionPrimitive,
        state: npt.NDArray[np.float64],
        dt: float,
        motion_model_function: Callable[
            [npt.NDArray[np.float64], DifferentialDriveControl, float], npt.NDArray[np.float64]
        ],
    ) -> List[npt.NDArray[np.float64]]: ...
    def compute_trajectory(
        self: DdcMotionPrimitive,
        state: npt.NDArray[np.float64],
        control_idx: int,
        dt: float,
        motion_model_function: Callable[
            [npt.NDArray[np.float64], DifferentialDriveControl, float], npt.NDArray[np.float64]
        ],
    ) -> npt.NDArray[np.float64]: ...

def load_ddc_motion_primitives_from_yaml(filename: str) -> List[DdcMotionPrimitive]: ...

class EnvironmentSe2(EnvironmentBase):
    class Setting(YamlableBase):
        time_step: float
        motion_primitives: List[DdcMotionPrimitive]
        num_orientations: int
        obstacle_threshold: float
        add_map_cost: bool
        map_cost_factor: float
        shape: npt.NDArray[np.float64]
    def __init__(
        self: EnvironmentSe2,
        grid_map: GridMapUnsigned2D,
        setting: Setting = None,
    ) -> None: ...
    @staticmethod
    def motion_model(
        metric_state: npt.NDArray[np.float64], control: DifferentialDriveControl, t: float
    ) -> npt.NDArray[np.float64]: ...

class EnvironmentAnchor(EnvironmentBase):
    def __init__(self: EnvironmentAnchor, environments: List[EnvironmentBase]) -> None: ...

class EnvironmentGridAnchor2D(EnvironmentAnchor):
    def __init__(
        self: EnvironmentGridAnchor2D, environments: List[EnvironmentBase], grid_map_info: GridMapInfo2D
    ) -> None: ...


class EnvironmentGridAnchor3D(EnvironmentAnchor):
    def __init__(
            self: EnvironmentGridAnchor3D, environments: List[EnvironmentBase], grid_map_info: GridMapInfo3D
    ) -> None: ...
