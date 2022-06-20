#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>
#include "ARAPMeshDeformation.h"

#include <iostream>
#include <thread>
#include <future>
#include <chrono>


typedef std::vector<std::vector<float>> VERTS;
typedef std::vector<VERTS> MESHES_VERTS;
typedef std::vector<MESHES_VERTS> SPLIT_MESHES_VERTS;

typedef std::vector<std::vector<int>> FACES;
typedef std::vector<FACES> MESHES_FACES;
typedef std::vector<MESHES_FACES> SPLIT_MESHES_FACES;

typedef std::vector<std::pair<int, std::vector<float>>> CONSTRAINTS;
typedef std::vector<CONSTRAINTS> MESHES_CONSTRAINTS;
typedef std::vector<MESHES_CONSTRAINTS> SPLIT_MESHES_CONSTRAINTS;


namespace py = pybind11;

template <typename T>
std::vector<std::vector<T>> split_vector(std::vector<T> v, int num_parts) {
	int n = v.size() / num_parts;

    // create an array of vectors to store the sub-vectors
    std::vector<std::vector<T>> vec = std::vector<std::vector<T>>();
 
    // each iteration of this loop process the next set of `n` elements
    // and store it in a vector at k'th index in `vec`
    for (int k = 0; k < num_parts; ++k) {
		typename std::vector<T>::const_iterator a = v.begin() + k*n;
		typename std::vector<T>::const_iterator b = v.begin() + ((k*n + n >= v.size()) ? v.size() : k*n + n);

		std::vector<T> new_vec(a, b);
		vec.push_back(new_vec);
    }
	
	return vec;
}

VERTS deform(VERTS vertices, FACES faces, CONSTRAINTS constraints) {
	BaseObject* vocfold = new BaseObject();
	for (std::vector<float> vertex : vertices)
		vocfold->add_vertex(BaseObject::Point(vertex.at(0), vertex.at(1), vertex.at(2)));

	for (std::vector<int> face : faces) {
		std::vector<BaseObject::VertexHandle> vhandle{BaseObject::VertexHandle(face.at(0)), BaseObject::VertexHandle(face.at(1)), BaseObject::VertexHandle(face.at(2))};
		vocfold->add_face(vhandle);
	}

	for(std::pair<int, std::vector<float>> constr : constraints) {
		//std::cout << "[ [" << constr.second.at(0) << "," << constr.second.at(1) << "," << constr.second.at(2) << "]," << constr.first << "]" << std::endl;
		auto bla = Constraint(Eigen::Vector3f(constr.second.data()), constr.first);
		//std::cout << Eigen::Vector3f(constr.second.data()) << std::endl;
		vocfold->addConstraint(bla);
		}

	ARAPMeshDeformation arap;
	arap.deform(*vocfold, *vocfold, vocfold->getConstraints(), 2);

	VERTS returnvec = VERTS();
	for (auto vertex : vocfold->vertices()) {
		Eigen::Vector3f point = vocfold->point(vertex);
		std::vector<float> veci{point[0], point[1], point[2]};
		returnvec.push_back(veci);
	}

	delete vocfold;
    return returnvec;
}


MESHES_VERTS deform_multiple(MESHES_VERTS meshes_vert, MESHES_FACES meshes_faces, MESHES_CONSTRAINTS meshes_constraints) {
	MESHES_VERTS returnvec = MESHES_VERTS();
	
	for (int i = 0; i < meshes_vert.size(); i++) {
		BaseObject* vocfold = new BaseObject();
		auto vertices = meshes_vert[i];
		auto faces = meshes_faces[i];
		auto constraints = meshes_constraints[i];

		for (auto vertex : vertices)
			vocfold->add_vertex(BaseObject::Point(vertex.at(0), vertex.at(1), vertex.at(2)));

		for (auto face : faces) {
			std::vector<BaseObject::VertexHandle> vhandle{BaseObject::VertexHandle(face.at(0)), BaseObject::VertexHandle(face.at(1)), BaseObject::VertexHandle(face.at(2))};
			vocfold->add_face(vhandle);
		}

		for(auto constr : constraints)
			vocfold->addConstraint(Constraint(Eigen::Vector3f(constr.second.data()), constr.first));

		ARAPMeshDeformation arap;
		arap.deform(*vocfold, *vocfold, vocfold->getConstraints(), 2);

		VERTS deformed_verts = VERTS();
		for (auto vertex : vocfold->vertices()) {
			Eigen::Vector3f point = vocfold->point(vertex);
			std::vector<float> veci{point[0], point[1], point[2]};
			deformed_verts.push_back(veci);
		}
		returnvec.push_back(deformed_verts);

		delete vocfold;
	}

    return returnvec;
}


MESHES_VERTS deform_async(MESHES_VERTS mesh_verts, MESHES_FACES mesh_faces, MESHES_CONSTRAINTS mesh_constraints, int num_threads) {
	SPLIT_MESHES_VERTS split_mesh_verts = split_vector(mesh_verts, num_threads);
	SPLIT_MESHES_FACES split_mesh_faces = split_vector(mesh_faces, num_threads);
	SPLIT_MESHES_CONSTRAINTS split_mesh_constraints = split_vector(mesh_constraints, num_threads);

	std::vector<std::shared_future<MESHES_VERTS>> futures = std::vector<std::shared_future<MESHES_VERTS>>();
	for (int i = 0; i < split_mesh_verts.size(); i++)
		futures.push_back(std::async(&deform_multiple, std::ref(split_mesh_verts[i]), std::ref(split_mesh_faces[i]), std::ref(split_mesh_constraints[i])));

	MESHES_VERTS deformed_mesh_verts = MESHES_VERTS();

	for (auto thread : futures) {
		MESHES_VERTS a = thread.get();
		deformed_mesh_verts.insert(deformed_mesh_verts.end(), a.begin(), a.end());
	}

	return deformed_mesh_verts;
}

PYBIND11_MODULE(FastARAP, m) {
    m.doc() = "";
    m.def("deform", &deform, "");
	m.def("deform_multiple", &deform_multiple, "");
	m.def("deform_async", &deform_async, "");
}