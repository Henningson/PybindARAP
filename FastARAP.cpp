#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>


#include <iostream>
#include <thread>
#include <future>
#include <chrono>

#include "ARAP.h"

namespace py = pybind11;

template <typename T>
std::vector<std::vector<T>> split_vector(std::vector<T> v, int num_parts) {
	int n = v.size() / num_parts;

    // create an array ovectors to store the sub-vectors
    std::vector<std::vector<T>> vec = std::vector<std::vector<T>>();
 
    // each iteration othis loop process the next set o`n` elements
    // and store it in a vector at k'th index in `vec`
    for (int k = 0; k < num_parts; ++k) {
		typename std::vector<T>::const_iterator a = v.begin() + k*n;
		typename std::vector<T>::const_iterator b = v.begin() + ((k*n + n >= v.size()) ? v.size() : k*n + n);

		std::vector<T> new_vec(a, b);
		vec.push_back(new_vec);
    }
	
	return vec;
}

VERTS deform(VERTS vertices, 
			 FACES faces, 
			 CONSTRAINTS constraints, 
			 IS_CONSTRAINED is_constraint, 
			 NEIGHBOURS neighborhood, 
			 int iterations, 
			 float weight) {

	BasicMesh mesh = {vertices, faces, constraints, is_constraint, neighborhood};

	ARAPDeformation a = ARAPDeformation();
	a.compute(mesh, mesh, iterations, weight);

	return mesh.vertices;
}


MESHES_VERTS deform_multiple(MESHES_VERTS meshes_vert, 
							 MESHES_FACES meshes_faces, 
							 MESHES_CONSTRAINTS meshes_constraints, 
							 MESHES_IS_CONSTRAINED is_constraint, 
							 MESHES_NEIGHBOURS neighborhood, 
							 int iterations, 
							 float weight) {

	MESHES_VERTS returnvec = MESHES_VERTS();
	for (int i = 0; i < meshes_vert.size(); i++) {
		BasicMesh mesh = {meshes_vert[i], meshes_faces[i], meshes_constraints[i], is_constraint[i], neighborhood[i]};
			
		ARAPDeformation a = ARAPDeformation();
		a.compute(mesh, mesh, 2, 0.5);
		returnvec.push_back(mesh.vertices);
	}

    return returnvec;
}




MESHES_VERTS deform_async(MESHES_VERTS meshes_vert, 
							 MESHES_FACES meshes_faces, 
							 MESHES_CONSTRAINTS meshes_constraints, 
							 MESHES_IS_CONSTRAINED is_constraint, 
							 MESHES_NEIGHBOURS neighborhood, 
							 int iterations, 
							 float weight,
							 int num_threads) {

	SPLIT_MESHES_VERTS split_mesh_verts = split_vector(meshes_vert, num_threads);
	SPLIT_MESHES_FACES split_mesh_faces = split_vector(meshes_faces, num_threads);
	SPLIT_MESHES_CONSTRAINTS split_mesh_constraints = split_vector(meshes_constraints, num_threads);
	SPLIT_MESHES_IS_CONSTRAINED split_mesh_is_constraints = split_vector(is_constraint, num_threads);
	SPLIT_MESHES_NEIGHBOURS split_mesh_neighbours = split_vector(neighborhood, num_threads);

	std::vector<std::shared_future<MESHES_VERTS>> futures = std::vector<std::shared_future<MESHES_VERTS>>();
	for (int i = 0; i < split_mesh_verts.size(); i++)
		futures.push_back(std::async(&deform_multiple, 
		std::ref(split_mesh_verts[i]), 
		std::ref(split_mesh_faces[i]), 
		std::ref(split_mesh_constraints[i]),
		std::ref(split_mesh_is_constraints[i]),
		std::ref(split_mesh_neighbours[i]), 
		iterations, 
		weight));

	MESHES_VERTS deformed_mesh_verts = MESHES_VERTS();

	for (auto thread : futures) {
		MESHES_VERTS a = thread.get();
		deformed_mesh_verts.insert(deformed_mesh_verts.end(), a.begin(), a.end());
	}

	return deformed_mesh_verts;
}

BasicMesh prepareBasicMesh() {
	VERTS verts = {{1.000000, -1.000000, -1.000000},
					{1.000000, -1.000000, 1.000000},
					{-1.000000, -1.000000, 1.000000},
					{-1.000000, -1.000000, -1.000000},
					{1.000000, 1.000000, -1.000000},
					{0.999999, 1.000000, 1.000001},
					{-1.000000, 1.000000, 1.000000},
					{-1.000000, 1.000000, -1.000000}};

	FACES faces = {{0, 1, 2},
					{0, 2, 3},
					{4, 7, 6},
					{4, 6, 5},
					{0, 4, 5},
					{0, 5, 1},
					{1, 5, 6},
					{1, 6, 2},
					{2, 6, 7},
					{2, 7, 3},
					{4, 0, 3},
					{4, 3, 7}};

	CONSTRAINTS constraints = {{3, {-1.0, -1.0, -1.0}}, {5, {2.0, 4.0, 2.0}}};

	IS_CONSTRAINED is_constrained = {false, false, false, true, false, true, false, false};

	NEIGHBOURS neighbors = {{1, 2, 3, 4, 5}, 
				{0, 2, 5, 6}, 
				{0, 1, 3, 6, 7}, 
				{0, 2, 4, 7}, 
				{0, 3, 5, 6, 7}, 
				{0, 1, 4, 6}, 
				{1, 2, 4, 5, 7}, 
				{2, 3, 4, 6}};

	return {verts, faces, constraints, is_constrained, neighbors};
}

PYBIND11_MODULE(FastARAP, m) {
    m.doc() = "";
    m.def("deform", &deform, "Deform a single mesh instance.");
	m.def("deform_multiple", &deform_multiple, "Deform multiple meshes inside a list.");
	m.def("deform_async", &deform_async, "Deform multiple meshes asynchronously.");
}
