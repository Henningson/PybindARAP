#include "BaseObject.h"

BaseObject::BaseObject() : OpenMesh::TriMesh_ArrayKernelT<ConstrainedTraits>() {
	this->name = "Object";
}

BaseObject::BaseObject(std::string name) : OpenMesh::TriMesh_ArrayKernelT<ConstrainedTraits>() {
	this->name = name;
}

BaseObject::BaseObject(const BaseObject& object) {

	int maxIndex = this->n_vertices();
	this->name = "Copy";
	//This is extremely inefficient.

	for (auto vert : object.vertices())
		this->add_vertex(object.point(vert));

	for (auto face : object.faces()) {
		std::vector<BaseObject::VertexHandle> new_face;
		for (auto vert : face.vertices())
			new_face.push_back(BaseObject::VertexHandle(vert.idx()));

		this->add_face(new_face);
	}

	for (auto constraint : object.getConstraints()) {
		this->constraints.push_back(Constraint(constraint.position, constraint.vertexIndex));
	}

	this->release_face_normals();
	this->release_vertex_normals();
	this->request_face_normals();
	this->update_normals();
	this->request_vertex_normals();
	this->update_normals();
	this->release_face_normals();
}

std::string BaseObject::getName() {
	return this->name;
}

void BaseObject::initKdTree() {
	kdTree = std::make_unique<KdTreeT>(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams());
	kdTree->buildIndex();
}

const std::vector<Constraint> BaseObject::getConstraints() const {
	return this->constraints;
}

bool BaseObject::hasConstraints() {
	return this->constraints.size() > 0;
}

void BaseObject::addConstraint(Constraint constraint) {
	this->constraints.push_back(constraint);
}

void BaseObject::removeLastNConstraints(int n) {
	if (n >= this->constraints.size())
		return;

	this->constraints.resize(this->constraints.size() - n);
}


std::vector<OpenMesh::ArrayKernel::VertexHandle> BaseObject::getKNearestPoints(const Eigen::Vector3f& point, const int k) {
	std::vector<OpenMesh::ArrayKernel::VertexHandle> results;
	results.reserve(k);

	std::vector<size_t> ret_indexes(k);
	std::vector<double> out_dists_sqr(k);
	nanoflann::KNNResultSet<double> resultSet(k);
	resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
	kdTree->findNeighbors(resultSet, &point[0], nanoflann::SearchParams());

	for (const auto& i : ret_indexes) {
		results.push_back(OpenMesh::ArrayKernel::VertexHandle(i));
	}

	return results;
}

void BaseObject::addObject(BaseObject* object) {
	int maxIndex = this->n_vertices();
	
	//This is extremely inefficient.

	for (auto vert : object->vertices())
		this->add_vertex(object->point(vert));

	for (auto face : object->faces()) {
		std::vector<BaseObject::VertexHandle> new_face;
		for (auto vert : face.vertices())
			new_face.push_back(BaseObject::VertexHandle(vert.idx() + maxIndex));

		this->add_face(new_face);
	}

	for (auto constraint : object->getConstraints())
		this->constraints.push_back(Constraint(constraint.position, constraint.vertexIndex + maxIndex));

	this->release_face_normals();
	this->release_vertex_normals();
	this->request_face_normals();
	this->update_normals();
	this->request_vertex_normals();
	this->update_normals();
	this->release_face_normals();
}