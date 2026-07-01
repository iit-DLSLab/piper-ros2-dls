## Installation with Conda

1. Install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64 or arm64 depending on your platform)

2. Create an environment using the file in the folder [installation](./installation):

    `conda env create -f installation/mamba_environment.yml`


3. Clone the other submodules:

    `git submodule update --init --recursive`

4. Activate the env and install the submodule

```bash
conda activate piper_ros2_env
pip install -e pyAgxArm/
```

