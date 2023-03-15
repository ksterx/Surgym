# Surgym

## Overview

Autonomous surgical robot with Reinforcement Learning.

## Installation

### 1. Prerequisites

- Ubuntu 20.04 or later
- NVIDIA driver version: 515 or later

### 2. Setup

#### - Install Omniverse Isaac Sim

- [Omniverse Platform for 3D Design Collaboration and Simulation | NVIDIA](https://www.nvidia.com/en-us/omniverse/) 

- [What Is Isaac Sim? &mdash; Omniverse Robotics documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)

#### - Set Python alias

```bash
alias omnipy=~/.local/share/ov/pkg/isaac_sim-*/python.sh
```

## Usage

### 1. Train RL model

```bash
omnipy train.py task=<TASK_NAME>
```
