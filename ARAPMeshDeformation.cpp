#include "ARAPMeshDeformation.h"

#include <Eigen/SparseLU>

ARAPMeshDeformation::ARAPMeshDeformation() {
	m_weightFit = 0.5f;
}

Eigen::SparseMatrix<float> ARAPMeshDeformation::computeMatrix(const BaseObject& mesh, float wFit) {
	// !!! TODO !!!
	//
	// Build System Matrix
	const size_t N = mesh.n_vertices();
	Eigen::SparseMatrix<float> mat(N, N);

	////////////!!! TODO !!!////////////
	for (auto& c_vh : mesh.vertices()) {
		const unsigned int valance = mesh.valence(c_vh);

		if (!mesh.data(c_vh).constrained) mat.insert(c_vh.idx(), c_vh.idx()) = valance;
		else								mat.insert(c_vh.idx(), c_vh.idx()) = valance + wFit;

		for (auto v_vh : mesh.vv_range(c_vh)) {
			mat.insert(v_vh.idx(), c_vh.idx()) = -1.0f;
		}
	}
	////////////!!! TODO !!!////////////

	mat.finalize();
	mat.makeCompressed();

	return mat;
}

void ARAPMeshDeformation::deform(BaseObject& mesh, const BaseObject& urshape, const std::vector<Constraint>& constraints, unsigned int numIterInner) {
	// Set all vertices to unconstrained
	for (auto& c_vh : mesh.vertices()) {
		mesh.data(c_vh).constrained = false;
	}

	// Set constraints
	for (auto& c : constraints) {
		unsigned int index = c.vertexIndex;

		OpenMesh::VertexHandle c_vh(index);
		mesh.data(c_vh).constrained = true;
	}

	// Compute matrix
	const Eigen::SparseMatrix<float, Eigen::ColMajor> mat = computeMatrix(mesh, m_weightFit);

	// Compute Factorization
	m_LU_of_A.analyzePattern(mat);
	m_LU_of_A.factorize(mat);

	if (m_LU_of_A.info() != Eigen::Success) {
		std::cout << "Factor Failed" << std::endl;
		exit(1);
	}

	deform_with_given_factorization(mesh, urshape, constraints, numIterInner, m_weightFit);
}

void ARAPMeshDeformation::deform_with_given_factorization(BaseObject& mesh, const BaseObject& urshape, const std::vector<Constraint>& constraints, unsigned int numInnerIter, float wFit) {
	// Build lookup table
	size_t numOfPoints = mesh.n_vertices();

	// Right Hand Side
	Eigen::VectorXf rhs_x(numOfPoints);
	Eigen::VectorXf rhs_y(numOfPoints);
	Eigen::VectorXf rhs_z(numOfPoints);

	// Vector to cache the best fitting rotations
	std::vector<Eigen::Matrix3f> rots(numOfPoints, Eigen::Matrix3f::Identity());

	// !!! TODO !!!
	//
	// Compute Right Hand Side and Best Fitting Rotations
	for (unsigned int ARAP_iter = 0; ARAP_iter < numInnerIter; ARAP_iter++) {
		// !!! TODO !!!
		//
		// Compute Right Hand Side
		for (auto& c_vh : mesh.vertices()) {
			Eigen::Vector3f rhs(0, 0, 0);

			////////////!!! TODO !!!////////////
			const Eigen::Vector3f& p_i = urshape.point(c_vh);

			for (auto v_vh : mesh.vv_range(c_vh)) {
				//! Right-hand side
				Eigen::Matrix3f rot = rots[c_vh.idx()] + rots[v_vh.idx()];

				const Eigen::Vector3f& p_j = urshape.point(v_vh);

				rhs += 0.5 * rot * (p_i - p_j);
			}
			////////////!!! TODO !!!////////////

			rhs_x[c_vh.idx()] = rhs[0];
			rhs_y[c_vh.idx()] = rhs[1];
			rhs_z[c_vh.idx()] = rhs[2];
		}

		for (auto& c : constraints) {
			Eigen::Vector3f rhs = Eigen::Vector3f::Zero();
			unsigned int idx = c.vertexIndex;

			////////////!!! TODO !!!////////////

			rhs = wFit * c.position;

			////////////!!! TODO !!!////////////

			rhs_x[idx] += rhs[0];
			rhs_y[idx] += rhs[1];
			rhs_z[idx] += rhs[2];
		}

		// Solve System
		Eigen::VectorXf x_x(numOfPoints);
		Eigen::VectorXf x_y(numOfPoints);
		Eigen::VectorXf x_z(numOfPoints);

		x_x = m_LU_of_A.solve(rhs_x);
		if (m_LU_of_A.info() != Eigen::Success) {
			std::cout << "Solve Failed" << std::endl;
		}

		x_y = m_LU_of_A.solve(rhs_y);
		if (m_LU_of_A.info() != Eigen::Success) {
			std::cout << "Solve Failed" << std::endl;
		}

		x_z = m_LU_of_A.solve(rhs_z);
		if (m_LU_of_A.info() != Eigen::Success) {
			std::cout << "Solve Failed" << std::endl;
		}

		// !!! TODO !!!
		//
		// Copy result to the mesh
		for (auto& c_vh : mesh.vertices()) {
			////////////!!! TODO !!!////////////			
			const Eigen::Vector3f pos(x_x[c_vh.idx()], x_y[c_vh.idx()], x_z[c_vh.idx()]);

			mesh.set_point(c_vh, pos);
			////////////!!! TODO !!!////////////
		}

		// Compute Best Fitting Rotations
		for (auto& c_vh : mesh.vertices()) {
			// !!! TODO !!!
			//
			// Build Covariance Matrix
			Eigen::Matrix3f COV = Eigen::Matrix3f::Zero();

			////////////!!! TODO !!!////////////
			for (auto v_vh : mesh.vv_range(c_vh)) {
				const auto& p_i = urshape.point(c_vh);
				const auto& p_j = urshape.point(v_vh);
				const auto& p_dash_i = mesh.point(c_vh);
				const auto& p_dash_j = mesh.point(v_vh);

				const auto d0 = p_i - p_j;
				const auto d1 = p_dash_i - p_dash_j;

				COV += d0 * d1.transpose();
			}
			////////////!!! TODO !!!////////////


			// Compute best aligning rotation
			Eigen::JacobiSVD<Eigen::Matrix3f> svd(COV, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3f U = svd.matrixU();
			Eigen::Matrix3f V = svd.matrixV();
			Eigen::Vector3f sigma = svd.singularValues();

			Eigen::Matrix3f VT = V.transpose();
			Eigen::Matrix3f rot_matrix = U * VT;

			// best rigid transformation may contain a reflection (det = -1)
			float det = rot_matrix.determinant();
			if (det < 0.0) { // trafo is reflection, pick second best!
				int sm_id = 0;
				float smallest = sigma[sm_id];
				for (unsigned int dd = 1; dd < 3; dd++) {
					if (sigma[dd] < smallest) {
						smallest = sigma[dd];
						sm_id = dd;
					}
				}

				// flip sign of entries in colums 'sm_id' in 'U'
				U.col(sm_id) *= -1;
				rot_matrix = U * VT;
			}

			rots[c_vh.idx()] = rot_matrix.transpose();
		}
	}

	//mesh.update_face_normals();
	//mesh.update_normals();
}
