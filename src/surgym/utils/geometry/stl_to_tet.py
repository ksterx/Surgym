import os
import subprocess

urdf_dir = "/workspace/data/assets/urdf"
model_name = input("Model Name: ")

# Make a new directory for the model if not already there
model_dir = os.path.join(urdf_dir, model_name)
if not os.path.exists(model_dir):
    os.makedirs(model_dir)

stl_path = os.path.join(model_dir, model_name + ".stl")
mesh_path = os.path.join(model_dir, model_name + ".mesh")

print(subprocess.run(["pwd"]))

os.chdir("/workspace/kit/fTetWild/build")
subprocess.run(["./FloatTetwild_bin", "-i", stl_path, "-o", mesh_path])


def convert_mesh_to_tet(mesh_file_path, tet_output_path):
    """Convert a .mesh file to a .tet file."""
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

    print("# Vertices, # Tetrahedra:", num_vertices, num_tetrahedra)

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


if __name__ == "__main__":
    convert_mesh_to_tet(mesh_path, os.path.join(model_dir, model_name + ".tet"))
