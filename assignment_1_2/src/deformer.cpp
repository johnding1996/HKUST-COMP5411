#include "deformer.h"
#include <iostream>

Deformer::Deformer() : mMesh(nullptr),
                       mCholeskySolver(nullptr) {
}

Deformer::~Deformer() {
	clear();
}

void Deformer::clear() {
	if (mCholeskySolver) {
		delete mCholeskySolver;
	}
	mCholeskySolver = nullptr;
	// Delete A and b
	if (A) {
		delete A;
	}
	A = nullptr;
	if (b) {
		delete b;
	}
	b = nullptr;
	mRoiList.clear();
}

void Deformer::setMesh(Mesh* mesh) {
	mMesh = mesh;
	clear();
	// Record the handle vertices
	for (Vertex* vert : mMesh->vertices()) {
		if (vert->flag() > 0 || vert->isBoundary()) {
			mRoiList.push_back(vert);
		}
	}
	// Build system matrix for deformation
	buildSystemMat();
}


void Deformer::buildSystemMat() {
	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Build the matrix of the linear system for 
	/* deformation and do factorization, in order
	/* to reuse and speed up in Deformer::deform().
	/* Handle vertices are maked by Vertex::flag() > 0
	/* Movements of the specified handle are already
	/* recorded in Vertex::position()
	/**********************************************/

	const std::vector< Vertex* > mVertexList = mMesh->vertices();

	// V: number of vertices
	// HV: number of constraint vertices (ROI)
	auto V = (int)mVertexList.size();
	auto HV = (int)mRoiList.size();

	Eigen::SparseMatrix< double > systemMat(V, V);

	A = new Eigen::SparseMatrix< double >(V + HV + 1, V);

	/*====== Programming Assignment 2 ======*/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html

	typedef Eigen::Triplet<double> T;
	std::vector< T > tripletList;
	for(int i=0; i< V; i++) {
		tripletList.push_back(T(i,i,1.0));
	}

	for (auto it=mVertexList.begin(); it!=mVertexList.end(); it++) {
		Vertex* curVertex = *it;
		// outHEdges: vector of all HEdges emerge from this vertex
		std::vector < HEdge* > outHEdges;
		int curIndex = curVertex->index();
		outHEdges.push_back(curVertex->halfEdge());
		while (outHEdges.back()->twin()->next()!=outHEdges[0]) {
			outHEdges.push_back(outHEdges.back()->twin()->next());
		}
		// calculate weights for each out HEdges
		std::vector < double > weights;
		for (auto it2=outHEdges.begin(); it2!=outHEdges.end(); it2++) {
			HEdge* outHEdge = *it2;
			Eigen::Vector3f HEdge11 = outHEdge->end()->position() - outHEdge->next()->end()->position();
			Eigen::Vector3f HEdge12 = outHEdge->next()->next()->end()->position() - outHEdge->next()->end()->position();
			double cotangent1 = HEdge11.dot(HEdge12) / HEdge11.cross(HEdge12).norm();
			HEdge* inHEdge = outHEdge->twin();
			Eigen::Vector3f HEdge21 = inHEdge->end()->position() - inHEdge->next()->end()->position();
			Eigen::Vector3f HEdge22 = inHEdge->next()->next()->end()->position() - inHEdge->next()->end()->position();
			double cotangent2 = HEdge21.dot(HEdge22) / HEdge21.cross(HEdge22).norm();
			weights.push_back((cotangent1 + cotangent2) / 2);
		}
		// calculate sum of weights
		double sumWeights = 0.0;
		std::for_each(weights.begin(), weights.end(), [&] (double w) {
			sumWeights += w;
		});
		// Assign Laplacian matrix elements according to the weights and neighboring relations
		auto it3=weights.begin();
		for (auto it2=outHEdges.begin(); it2!=outHEdges.end(); it2++) {
			int nextIndex = (*it2)->end()->index();
			double w = *it3;
			tripletList.push_back(T(curIndex, nextIndex, - w / sumWeights));
			it3++;
		}
	}

	// constraint part of extended Laplacian matrix A
	tripletList.push_back(T(V , mVertexList[0]->index(), 1.0));
	int count = 0;
	for (auto it=mRoiList.begin(); it!=mRoiList.end(); it++) {
		Vertex* curVertex = *it;
		tripletList.push_back(T(V + count + 1, curVertex->index(), 1.0));
		count++;
	}

	A->setFromTriplets(tripletList.begin(), tripletList.end());
	systemMat = A->transpose() * (*A);

	// Compute Laplacian coordinates
	b = new Eigen::MatrixX3d (V+HV+1, 3);
	Eigen::MatrixX3d cord(V, 3);
	for (auto it=mVertexList.begin(); it!=mVertexList.end(); it++) {
		Vertex* curVertex = *it;
		for (int i=0; i<3; i++) {
			cord(curVertex->index(), i) = curVertex->position()[i];
		}
	}
	*b = (*A) * cord;

	// Do factorization
	if (systemMat.nonZeros() > 0) {
		mCholeskySolver = new Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >();
		mCholeskySolver->compute(systemMat);
		if (mCholeskySolver->info() != Eigen::Success) {
			// Decomposition failed
			std::cout << "Sparse decomposition failed\n";
		} else {
			std::cout << "Sparse decomposition succeeded\n";
		}
	}
}

void Deformer::deform() {
	if (mCholeskySolver == nullptr || mMesh == nullptr) {
		return;
	}

	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* This is the place where the editing techniques 
	/* take place.
	/* Solve for the new vertex positions after the 
	/* specified handles move using the factorized
	/* matrix from Deformer::buildSystemMat(), i.e.,
	/* mCholeskySolver defined in deformer.h
	/**********************************************/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html


	/*====== Programming Assignment 2 ======*/

	const std::vector< Vertex* > mVertexList = mMesh->vertices();
	auto V = (int)mVertexList.size();

	// update constraints in old Laplacian coordinates b
	for (int i=0; i<3; i++) {
		(*b)(V, i) = mVertexList[0]->position()[i];
	}
	int count = 0;
	for (auto it=mRoiList.begin(); it!=mRoiList.end(); it++) {
		Vertex* curVertex = *it;
		for (int i=0; i<3; i++) {
			(*b)(V + count + 1, i) = curVertex->position()[i];
		}
		count++;
	}

	// solve new coordinates
	Eigen::MatrixX3d x(V, 3);
	x = mCholeskySolver->solve(A->transpose() * (*b));

	// update new coordinates
	for (auto it=mVertexList.begin(); it!=mVertexList.end(); it++) {
		Vertex* curVertex = *it;
		Eigen::Vector3f newPosition;
		for (int i=0; i<3; i++) {
			newPosition[i] = x(curVertex->index(), i);
		}
		curVertex->setPosition(newPosition);
	}

}
