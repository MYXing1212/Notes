### 如何运行Task
```bash
python scripts/skrl/train.py --task=Template-Isaac-Lab-Tutorial-Direct-v0

./isaaclab.bat -p ./source/IsaacLabTutorial/scripts/skrl/train.py --task Template-Isaac-Lab-Tutorial-Direct-v0 --num_envs 16
```

### Task在哪里定义的
文件路径在

E:\Isaac\IssacLab\source\IsaacLabTutorial\source\isaac_lab_tutorial\isaac_lab_tutorial\tasks\direct\isaac_lab_tutorial\__init__.py
```python
# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0  

import gymnasium as gym 

from . import agents  

##
# Register Gym environments.
##  

gym.register(
    id="Template-Isaac-Lab-Tutorial-Direct-v0",
    entry_point=f"{__name__}.isaac_lab_tutorial_env:IsaacLabTutorialEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaac_lab_tutorial_env_cfg:IsaacLabTutorialEnvCfg",
        "skrl_amp_cfg_entry_point": f"{agents.__name__}:skrl_amp_cfg.yaml",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
```