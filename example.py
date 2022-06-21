import sys
sys.path.append("build/")

import pickle
import numpy as np
import FastARAP
import Timer

from tqdm import tqdm


def load_dict(filename_):
    with open(filename_, 'rb') as f:
        ret_di = pickle.load(f)
    return ret_di


def reorder_faces(vertices, faces):
    verts_centroid = np.sum(vertices, axis=0) / vertices.shape[0]
    new_faces = list()
    for face in faces:
        vert0 = vertices[face[0]]
        vert1 = vertices[face[1]]
        vert2 = vertices[face[2]]

        AB = -vert0 + vert1
        AC = -vert0 + vert2
        normal = np.cross(AB, AC)
        normal = normal / np.linalg.norm(normal)
        
        triangle_center = (vert0 + vert1 + vert2) / 3.0
        direction = -verts_centroid + triangle_center
        direction = direction / np.linalg.norm(direction)

        if np.dot(direction, normal) > 0.0:
            new_faces.append([face[0], face[1], face[2]])
        else:
            new_faces.append([face[2], face[1], face[0]])
    
    return new_faces



verts = [[1.0, 1.0, 0.0], [-1.0, 1.0, 1.0], [-1.0, -1.0, 0.0], [1.0, -1.0, 1.0]]
faces = [[0, 1, 2], [0, 3, 2]]
constraints = [[0, [1.0, 1.0, 1.0]]]
is_constraint = [True, False, False, False]
neighbours = [[1, 2, 3], [0, 2], [0, 1, 3], [0, 2]]

more_verts = list()
more_faces = list()
more_constraints = list()
more_is_constraint = list()
more_neighbours = list()
for i in range(500):
    more_verts.append(verts)
    more_faces.append(faces)
    more_constraints.append(constraints)
    more_is_constraint.append(is_constraint)
    more_neighbours.append(neighbours)


print("----- Single ARAP Iteration -----")
timer = Timer.Timer()
timer.start()
deformed_verts = FastARAP.deform(verts, faces, constraints, is_constraint, neighbours, 2, 0.5)
timer.stop()
print(str(timer))


print("----- Sequential ARAP -----")
timer = Timer.Timer()
timer.start()
deformed_verts = FastARAP.deform_multiple(more_verts, more_faces, more_constraints, more_is_constraint, more_neighbours, 2, 0.5)
timer.stop()
print(str(timer) + " for {0} Meshes".format(len(more_verts)))


print("----- Parallel ARAP -----")
num_threads = 8
timer = Timer.Timer()
timer.start()
deformed_verts = FastARAP.deform_async(more_verts, more_faces, more_constraints, more_is_constraint, more_neighbours, 2, 0.5, num_threads)
timer.stop()
print(str(timer) + " for {0} Meshes using {1} Threads.".format(len(more_verts), num_threads))