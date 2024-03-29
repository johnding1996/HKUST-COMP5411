#ifndef DEFORMER_H
#define DEFORMER_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "mesh.h"

// Deform mesh using Laplacian coordinates
class Deformer {
public:
	Deformer();
	~Deformer();

	void setMesh(Mesh* mesh);

	/*====== Programming Assignment 2 ======*/
	// This is the place where the editing techniques take place
	void deform();
	/*====== Programming Assignment 2 ======*/

private:
	/*====== Programming Assignment 2 ======*/
	// Build left hand side matrix and pre-factorize it
	void buildSystemMat();
	/*====== Programming Assignment 2 ======*/

	void clear();

	Mesh* mMesh;
	std::vector< Vertex* > mRoiList;
	// Solver for pre-factorizing the system matrix of the deformation
	Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >* mCholeskySolver;

    // A is the extended Laplacian matrix
	Eigen::SparseMatrix< double >* A;
    // b is the extended Laplacian coordinates (which also includes constraints)
	Eigen::MatrixX3d* b;
};

#endif // DEFORMER_H
