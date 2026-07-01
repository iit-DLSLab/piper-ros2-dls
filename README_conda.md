## Installation with Conda

1. install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64 or arm64 depending on your platform)

2. create an environment using the file in the folder [installation](https://github.com/iit-DLSLab/piper-ros2-dls/tree/main/installation):

    `conda env create -f installation/mamba_environment.yml`


3. clone the other submodules:

    `git submodule update --init --recursive`

4. activate the env and install the submodule

```bash
conda activate piper_ros2_env
pip install -e pyAgxArm/
```

