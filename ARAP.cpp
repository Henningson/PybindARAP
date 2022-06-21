#include "ARAP.h"

ARAPDeformation::ARAPDeformation() {
}

Eigen::MatrixXf ARAPDeformation::buildSystemMatrix(const BasicMesh& mesh, float weight) {

	const size_t N = mesh.vertices.size();
	Eigen::MatrixXf systemMatrix = Eigen::MatrixXf::Zero(N, N);

	for (int i = 0; i < N; i++) {
		const unsigned int valence = mesh.neighborhood.at(i).size();

		if (!mesh.is_constrained[i]) 
			systemMatrix(i, i) = valence;
		else
			systemMatrix(i, i) = valence + weight;

		for (int neighbour : mesh.neighborhood.at(i))
			systemMatrix(neighbour, i) = -1.0f;
	}

	return systemMatrix;
}

void ARAPDeformation::compute(BasicMesh& mesh, const BasicMesh& basemesh, unsigned int iterations, float weight) {
	auto systemMatrix = this->buildSystemMatrix(mesh, weight);
	this->LU = systemMatrix.fullPivLu();
	deform(mesh, basemesh, iterations, weight);
}

void ARAPDeformation::deform(BasicMesh& mesh, const BasicMesh& baseshape, unsigned int iterations, float weight) {
	size_t numOfPoints = mesh.vertices.size();

	Eigen::VectorXf rhs_x(numOfPoints);
	Eigen::VectorXf rhs_y(numOfPoints);
	Eigen::VectorXf rhs_z(numOfPoints);

	std::vector<Eigen::Matrix3f> rots(numOfPoints, Eigen::Matrix3f::Identity());

	for (unsigned int iter = 0; iter < iterations; iter++) {
		for (int i = 0; i < numOfPoints; i++) {
			Eigen::Vector3f rhs(0, 0, 0);
			const Eigen::Vector3f& p_i = Eigen::Vector3f(baseshape.vertices.at(i).data());

			for (int neighbour : mesh.neighborhood.at(i)) {
				Eigen::Matrix3f rot = rots[i] + rots[neighbour];

				const Eigen::Vector3f& p_j = Eigen::Vector3f(baseshape.vertices.at(neighbour).data());

				rhs += 0.5 * rot * (p_i - p_j);
			}
			rhs_x[i] = rhs[0];
			rhs_y[i] = rhs[1];
			rhs_z[i] = rhs[2];
		}

		for (auto& c : mesh.constraints) {
			Eigen::Vector3f rhs = Eigen::Vector3f::Zero();
			unsigned int idx = c.first;

			rhs = weight * Eigen::Vector3f(c.second.data());

			rhs_x[idx] += rhs[0];
			rhs_y[idx] += rhs[1];
			rhs_z[idx] += rhs[2];
		}

		// Solve System
		Eigen::VectorXf x_x(numOfPoints);
		Eigen::VectorXf x_y(numOfPoints);
		Eigen::VectorXf x_z(numOfPoints);

		x_x = this->LU.solve(rhs_x);
		x_y = this->LU.solve(rhs_y);
		x_z = this->LU.solve(rhs_z);

		for (int i = 0; i < numOfPoints; i++)
			mesh.vertices.at(i) = {x_x[i], x_y[i], x_z[i]};


		for (int i = 0; i < numOfPoints; i++) {
			Eigen::Matrix3f COV = Eigen::Matrix3f::Zero();

			for (int neighbour : mesh.neighborhood.at(i)) {
				const auto& p_i = Eigen::Vector3f(baseshape.vertices.at(i).data());
				const auto& p_j = Eigen::Vector3f(baseshape.vertices.at(neighbour).data());
				const auto& p_dash_i = Eigen::Vector3f(mesh.vertices.at(i).data());
				const auto& p_dash_j = Eigen::Vector3f(mesh.vertices.at(neighbour).data());

				const auto d0 = p_i - p_j;
				const auto d1 = p_dash_i - p_dash_j;

				COV += d0 * d1.transpose();
			}

			// Compute best aligning rotation
			Eigen::JacobiSVD<Eigen::Matrix3f> svd(COV, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3f U = svd.matrixU();
			Eigen::Matrix3f V = svd.matrixV();
			Eigen::Vector3f sigma = svd.singularValues();

			Eigen::Matrix3f VT = V.transpose();
			Eigen::Matrix3f rot_matrix = U * VT;

			float det = rot_matrix.determinant();
			if (det < 0.0) {
				int sm_id = 0;
				float smallest = sigma[sm_id];
				for (unsigned int dd = 1; dd < 3; dd++) {
					if (sigma[dd] < smallest) {
						smallest = sigma[dd];
						sm_id = dd;
					}
				}
				U.col(sm_id) *= -1;
				rot_matrix = U * VT;
			}

			rots[i] = rot_matrix.transpose();
		}
	}
}
