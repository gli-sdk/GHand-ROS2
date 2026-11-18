#!/usr/bin/env python3
import copy
import sys
from pathlib import Path
import yaml
source = yaml.safe_load(Path(sys.argv[1]).read_text())
manager = source["controller_manager"]["ros__parameters"]
manager.pop("joint_trajectory_controller")
controller = source.pop("joint_trajectory_controller")
for side in ("left", "right"):
    name = side + "_joint_trajectory_controller"
    manager[name] = {"type": "joint_trajectory_controller/JointTrajectoryController"}
    parameters = copy.deepcopy(controller["ros__parameters"])
    names = parameters["joints"]
    parameters["joints"] = [side + "_" + joint for joint in names]
    for key in ("constraints", "gains"):
        if key in parameters:
            parameters[key] = {(side + "_" + k if k in names else k): v for k, v in parameters[key].items()}
    source[name] = {"ros__parameters": parameters}
Path(sys.argv[2]).write_text(yaml.safe_dump(source, sort_keys=False))
