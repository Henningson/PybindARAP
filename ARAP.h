#pragma once

#include <Eigen/Dense>

typedef std::vector<std::vector<float>> VERTS;
typedef std::vector<VERTS> MESHES_VERTS;
typedef std::vector<MESHES_VERTS> SPLIT_MESHES_VERTS;

typedef std::vector<std::vector<int>> FACES;
typedef std::vector<FACES> MESHES_FACES;
typedef std::vector<MESHES_FACES> SPLIT_MESHES_FACES;

typedef std::vector<std::pair<int, std::vector<float>>> CONSTRAINTS;
typedef std::vector<CONSTRAINTS> MESHES_CONSTRAINTS;
typedef std::vector<MESHES_CONSTRAINTS> SPLIT_MESHES_CONSTRAINTS;

typedef std::vector<std::vector<int>> NEIGHBOURS;
typedef std::vector<NEIGHBOURS> MESHES_NEIGHBOURS;
typedef std::vector<MESHES_NEIGHBOURS> SPLIT_MESHES_NEIGHBOURS;

typedef std::vector<bool> IS_CONSTRAINED;
typedef std::vector<IS_CONSTRAINED> MESHES_IS_CONSTRAINED;
typedef std::vector<MESHES_IS_CONSTRAINED> SPLIT_MESHES_IS_CONSTRAINED;

struct BasicMesh {
	VERTS vertices;
	FACES faces;
	CONSTRAINTS constraints;
	IS_CONSTRAINED is_constrained;
	NEIGHBOURS neighborhood;
};

class ARAPDeformation {
public:
	ARAPDeformation();
	void compute(BasicMesh& mesh, const BasicMesh& gtShape, unsigned int iterations, float weight);
	Eigen::MatrixXf matti;

private:
	static Eigen::MatrixXf buildSystemMatrix(const BasicMesh& mesh, float weight);
	void deform(BasicMesh& mesh, const BasicMesh& baseMesh, unsigned int iterations, float weight);

	Eigen::FullPivLU<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> LU;
};
