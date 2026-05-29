## Overview

A wrapper around the `piper_sdk` repository that provides a script for communicating with DLS controllers.

## Prerequisites

- Install system dependencies:

```bash
sudo apt update
sudo apt install -y can-utils
```

- Clone this repository including submodules:

```bash
git clone --recurse-submodules git@github.com:iit-DLSLab/piper-ros2-dls.git
cd piper-ros2-dls
```

## Python environment (uv)

This project includes instructions using the `uv` manager.
Adapt these commands to your environment if you prefer `venv`, `pipenv`, or `conda`.

Create an environment and activate it:

```bash
uv venv
source .venv/bin/activate
```

Install the `piper_sdk` package in editable mode and other Python dependencies:

```bash
uv pip install -e ./piper_sdk
uv pip install pyyaml numpy
```

## Usage

Activate CAN-bus communication for the arm:

```bash
./piper_sdk/piper_sdk/can_activate.sh
```

Run the DLS2 hardware-abstraction-layer launcher script:

```bash
python3 launch_piper_hal.py
```

## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi).
