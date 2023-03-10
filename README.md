# Surgym

## Overview

Autonomous surgical robot with Reinforcement Learning.

## Installation

### 1. Prerequisites

- Ubuntu 20.04 or later
- Python 3.8 (Anaconda is recommended)
- NVIDIA driver version: 515 or later

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

#### - Clone [fTetWild](https://github.com/wildmeshing/fTetWild) to `surgym/src/surgym/pipelines/modeling/`

In the modeling of pipelines directory, clone the fTetWild repository during running the docker container with the following command:

```bash
cd surgym/src/surgym/pipelines/modeling
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
cd src/surgym
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


## Rules and guidelines

In order to get the best out of the template:

* Don't remove any lines from the `.gitignore` file we provide
* Make sure your results can be reproduced by following a [data engineering convention](https://kedro.readthedocs.io/en/stable/faq/faq.html#what-is-data-engineering-convention)
* Don't commit data to your repository
* Don't commit any credentials or your local configuration to your repository. Keep all your credentials and local configuration in `conf/local/`

## How to install dependencies

Declare any dependencies in `src/requirements.txt` for `pip` installation and `src/environment.yml` for `conda` installation.

To install them, run:

```
pip install -r src/requirements.txt
```

## How to run your Kedro pipeline

You can run your Kedro project with:

```
kedro run
```

## How to test your Kedro project

Have a look at the file `src/tests/test_run.py` for instructions on how to write your tests. You can run your tests as follows:

```
kedro test
```

To configure the coverage threshold, go to the `.coveragerc` file.

## Project dependencies

To generate or update the dependency requirements for your project:

```
kedro build-reqs
```

This will `pip-compile` the contents of `src/requirements.txt` into a new file `src/requirements.lock`. You can see the output of the resolution by opening `src/requirements.lock`.

After this, if you'd like to update your project requirements, please update `src/requirements.txt` and re-run `kedro build-reqs`.

[Further information about project dependencies](https://kedro.readthedocs.io/en/stable/kedro_project_setup/dependencies.html#project-specific-dependencies)

## How to work with Kedro and notebooks

> Note: Using `kedro jupyter` or `kedro ipython` to run your notebook provides these variables in scope: `context`, `catalog`, and `startup_error`.
>
> Jupyter, JupyterLab, and IPython are already included in the project requirements by default, so once you have run `pip install -r src/requirements.txt` you will not need to take any extra steps before you use them.

### Jupyter
To use Jupyter notebooks in your Kedro project, you need to install Jupyter:

```
pip install jupyter
```

After installing Jupyter, you can start a local notebook server:

```
kedro jupyter notebook
```

### JupyterLab
To use JupyterLab, you need to install it:

```
pip install jupyterlab
```

You can also start JupyterLab:

```
kedro jupyter lab
```

### IPython
And if you want to run an IPython session:

```
kedro ipython
```

### How to convert notebook cells to nodes in a Kedro project
You can move notebook code over into a Kedro project structure using a mixture of [cell tagging](https://jupyter-notebook.readthedocs.io/en/stable/changelog.html#release-5-0-0) and Kedro CLI commands.

By adding the `node` tag to a cell and running the command below, the cell's source code will be copied over to a Python file within `src/<package_name>/nodes/`:

```
kedro jupyter convert <filepath_to_my_notebook>
```
> *Note:* The name of the Python file matches the name of the original notebook.

Alternatively, you may want to transform all your notebooks in one go. Run the following command to convert all notebook files found in the project root directory and under any of its sub-folders:

```
kedro jupyter convert --all
```

### How to ignore notebook output cells in `git`
To automatically strip out all output cell contents before committing to `git`, you can run `kedro activate-nbstripout`. This will add a hook in `.git/config` which will run `nbstripout` before anything is committed to `git`.

> *Note:* Your output cells will be retained locally.

## Package your Kedro project

[Further information about building project documentation and packaging your project](https://kedro.readthedocs.io/en/stable/tutorial/package_a_project.html)
