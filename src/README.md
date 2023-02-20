# Autonomous Forceps Manipulation in Surgery

## Installation

### 1. Prerequisites

- Ubuntu 20.04
- Python 3.8
- NVIDIA driver version: 470 or later

### 2. Setup

#### - Install Isaac Gym to the root directory

See [Isaac Gym Installation Guide](https://developer.nvidia.com/isaac-gym).

#### - Install in a Docker container

In the root directory of the repository, run the following command:

```bash
bash docker/build.sh
```

Then, run the following command to start the container:

```bash
bash docker/run.sh <display>
```

You can find `<display>` number by `xpyinfo` command. Most likely, it is `0`.

#### - Clone [fTetWild](https://github.com/wildmeshing/fTetWild) to `kxbot-pipeline/src/kxbot/pipelines/modeling/`

In the modeling of pipelines directory, clone the fTetWild repository during running the docker container with the following command:

```bash
cd /workspace/kxbot-pipeline/src/kxbot/pipelines/modeling
git clone https://github.com/wildmeshing/fTetWild.git
```

and then, run the following command to install the fTetWild:

```bash
cd fTetWild
mkdir build
cd build
cmake ..
make
```

#### - Deprecated Method (NOT RECOMMENDED)

Declare any dependencies in `src/requirements.txt` for `pip` installation and `src/environment.yml` for `conda` installation.

To install them, run:

```bash
kedro install
```

## Usage

### 1. Docker

To run the container in `envs/isaacgym/docker`, run the following command:

```bash
cd /workspace/envs/isaacgym/docker
docker-compose up
```

### 2. Workflow

### Modeling Workflow

#### - Create a new 3D model

In blender, create a 3D model and save it to `data/assets/<model_name>/<model_name>.stl` (**Y-up**).

#### - Convert `.stl` to `.tet` and generate `.urdf`

To execute the pipeline, run in the `kxbot-pipeline` folder:

```bash
cd /workspace/kxbot-pipeline
kedro run --pipeline=model --params model_name:<model_name>
```

## Development

### 1. Workflow

#### - Add a new workflow

```bash
kedro pipeline create <pipeline_name>
```

See [kedro documentation](https://kedro.readthedocs.io/en/stable/06_nodes_and_pipelines/03_modular_pipelines.html).

### 2. Project dependencies

To generate or update the dependency requirements for your project:

```bash
kedro build-reqs
```

This will copy the contents of `src/requirements.txt` into a new file `src/requirements.in` which will be used as the source for `pip-compile`. You can see the output of the resolution by opening `src/requirements.txt`.

After this, if you'd like to update your project requirements, please update `src/requirements.in` and re-run `kedro build-reqs`.

[Further information about project dependencies](https://kedro.readthedocs.io/en/stable/04_kedro_project_setup/01_dependencies.html#project-specific-dependencies)
