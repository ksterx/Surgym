# Copyright 2021 QuantumBlack Visual Analytics Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
# NONINFRINGEMENT. IN NO EVENT WILL THE LICENSOR OR OTHER CONTRIBUTORS
# BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF, OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# The QuantumBlack Visual Analytics Limited ("QuantumBlack") name and logo
# (either separately or in combination, "QuantumBlack Trademarks") are
# trademarks of QuantumBlack. The License does not grant you any right or
# license to the QuantumBlack Trademarks. You may not use the QuantumBlack
# Trademarks or any confusingly similar mark as a trademark for your product,
# or use the QuantumBlack Trademarks in any other manner that might cause
# confusion in the marketplace, including but not limited to in advertising,
# on websites, or on software.
#
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path


def get_path(model_name):
    asset_dir = Path("/workspace/data/assets")  # TODO: Read from config
    model_dir = asset_dir / model_name

    path_wo_ext = str(model_dir / model_name)
    stl_file_path = path_wo_ext + ".stl"
    mesh_file_path = path_wo_ext + ".mesh"
    tet_file_path = path_wo_ext + ".tet"

    # Quit if the model directory does not exist or if the .stl file does not exist
    if not model_dir.exists():
        raise FileNotFoundError(f"Model directory {model_dir} does not exist.")
    if not Path(stl_file_path).exists():
        raise FileNotFoundError(f"STL file {stl_file_path} does not exist.")

    path_dict = {
        "path_wo_ext": path_wo_ext,
        "stl_file_path": stl_file_path,
        "mesh_file_path": mesh_file_path,
        "tet_file_path": tet_file_path,
    }

    return path_dict


def convert_stl_to_mesh(stl_file_path: str, mesh_file_path: str):
    """Convert a .stl file to a .mesh file."""

    # Run fTetWild (See https://github.com/wildmeshing/fTetWild.git)
    os.chdir("/workspace/kxbot-pipeline/src/kxbot/pipelines/modeling/fTetWild/build")
    subprocess.run(
        ["./FloatTetwild_bin", "-i", stl_file_path, "-o", mesh_file_path], timeout=60
    )

    # Delete unused files
    while not Path(stl_file_path).exists():
        time.sleep(1)
    unused_file_exts = [".mesh__sf.obj", ".mesh__tracked_surface.stl", ".mesh_.csv"]
    for ext in unused_file_exts:
        unused_file_path = mesh_file_path.replace(".mesh", ext)
        os.remove(unused_file_path)


def convert_mesh_to_tet(mesh_file_path: str, tet_output_path: str):
    """Convert a .mesh file to a .tet file."""

    while True:
        if Path(mesh_file_path).exists():
            break
        time.sleep(1)
        print("Waiting for mesh file to be generated...")

    mesh_file = open(mesh_file_path, "r")
    tet_output = open(tet_output_path, "w")

    mesh_lines = list(mesh_file)
    mesh_lines = [line.strip("\n") for line in mesh_lines]
    vertices_start = mesh_lines.index("Vertices")
    num_vertices = mesh_lines[vertices_start + 1]

    vertices = mesh_lines[vertices_start + 2 : vertices_start + 2 + int(num_vertices)]

    tetrahedra_start = mesh_lines.index("Tetrahedra")
    num_tetrahedra = mesh_lines[tetrahedra_start + 1]
    tetrahedra = mesh_lines[
        tetrahedra_start + 2 : tetrahedra_start + 2 + int(num_tetrahedra)
    ]

    print(f"# Vertices:   {num_vertices}")
    print(f"# Tetrahedra: {num_tetrahedra}")

    # Write to tet output
    tet_output.write("# Tetrahedral mesh generated using\n\n")
    tet_output.write("# " + num_vertices + " vertices\n")
    for v in vertices:
        tet_output.write("v " + v + "\n")
    tet_output.write("\n")
    tet_output.write("# " + num_tetrahedra + " tetrahedra\n")
    for t in tetrahedra:
        line = t.split(" 0")[0]
        line = line.split(" ")
        line = [str(int(k) - 1) for k in line]
        l_text = " ".join(line)
        tet_output.write("t " + l_text + "\n")

def genarate_urdf(path_wo_ext, **kwargs):
    model_name = str(Path(path_wo_ext).name)
    tree = ET.parse("/workspace/data/assets/original.urdf")
    root = tree.getroot()
    root.attrib["name"] = model_name
    for child in root:
        if child.attrib["name"] == "original":
            for elem in child.find("fem"):
                if elem.tag == "tetmesh":
                    elem.attrib["filename"] = model_name + ".tet"
                child.attrib["name"] = model_name
        if child.attrib["name"] == "attach":
            for elem in child:
                if elem.tag == "child":
                    elem.attrib["link"] = model_name
    tree.write(f"{path_wo_ext}.urdf")

# TODO: Register asset to config.py
