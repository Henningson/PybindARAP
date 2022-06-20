#pragma once

// Includes OpenMesh
#define NOMINMAX

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Geometry/EigenVectorT.hh>

#include "nanoflann.hpp"

struct ConstrainedTraits : OpenMesh::DefaultTraits {
	using Point = Eigen::Vector3f;
	using Normal = Eigen::Vector3f;

	using Scalar = double;

	template <class Base, class Refs> struct VertexT : public Base {
	public:
		bool constrained;
	};
};


class Constraint {
public:
	Constraint() : position(Eigen::Vector3f::Zero()), vertexIndex(-1) {}
	Constraint(const Eigen::Vector3f& pos, unsigned int vertexIndex) : position(pos), vertexIndex(vertexIndex){}

	const Eigen::Vector3f position;
	const unsigned int vertexIndex;
};

class BaseObject : public OpenMesh::TriMesh_ArrayKernelT<ConstrainedTraits> {
	using KdTreeT = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, BaseObject>, BaseObject, 3>;
public:
	BaseObject();
	BaseObject(std::string name);
	BaseObject(const BaseObject & object);

	std::string getName();

	std::vector<OpenMesh::ArrayKernel::VertexHandle> getKNearestPoints(const Eigen::Vector3f& point, const int k);
	void addObject(BaseObject* object);

	//For nanoflann
	void initKdTree();
	inline size_t kdtree_get_point_count() const { return this->n_vertices(); }
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const {

		return this->point(OpenMesh::ArrayKernel::VertexHandle(idx))[dim];
	}

	template <class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }

	const std::vector<Constraint> getConstraints() const;
	bool hasConstraints();
	void addConstraint(Constraint constraint);
	void removeLastNConstraints(int n);

protected:
	std::unique_ptr<KdTreeT> kdTree;
	std::string name;
	std::vector<Constraint> constraints;
};