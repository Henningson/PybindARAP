#pragma once

#include "BaseObject.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/SparseLU>

class ARAPMeshDeformation {
public:
	ARAPMeshDeformation();

	void deform(BaseObject& mesh, const BaseObject& urshape, const std::vector<Constraint>& constraints, unsigned int numIterInner);

private:
	static Eigen::SparseMatrix<float> computeMatrix(const BaseObject& mesh, float wFit);
	void deform_with_given_factorization(BaseObject& mesh, const BaseObject& urshape, const std::vector<Constraint>& constraints, unsigned int numInnerIter, float wFit);

	float m_weightFit;
	Eigen::SparseLU<Eigen::SparseMatrix<float, Eigen::ColMajor>, Eigen::COLAMDOrdering<int>> m_LU_of_A;
};
