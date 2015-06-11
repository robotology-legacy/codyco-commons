/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Francesco Romano
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "MathUtils.h"
#include <Eigen/SVD>

namespace codyco {

    namespace math {
        
        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           double tolerance,
                           unsigned int computationOptions)

        {
            Eigen::JacobiSVD<typename Eigen::MatrixXd::PlainObject> svdDecomposition(A.rows(), A.cols());
            pseudoInverse(A, svdDecomposition, Apinv, tolerance, computationOptions);
        }
        
        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           double tolerance,
                           unsigned int computationOptions)
        {
            using namespace Eigen;
            int nullSpaceRows = -1, nullSpaceCols = -1;
            pseudoInverse(A, svdDecomposition, Apinv, tolerance,
                          (double*)0, nullSpaceRows, nullSpaceCols, computationOptions);
        }

        void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                           Eigen::Ref<Eigen::MatrixXd> Apinv,
                           double tolerance,
                           double * nullSpaceBasisOfA,
                           int &nullSpaceRows, int &nullSpaceCols,
                           unsigned int computationOptions)
        {
            using namespace Eigen;
            
            if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
            svdDecomposition.compute(A, computationOptions);
            
            JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
            int rank = 0;
            for (int idx = 0; idx < singularValues.size(); idx++) {
                if (tolerance > 0 && singularValues(idx) > tolerance) {
                    singularValues(idx) = 1.0 / singularValues(idx);
                    rank++;
                } else {
                    singularValues(idx) = 0.0;
                }
            }
            Apinv = svdDecomposition.matrixV().leftCols(rank) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(rank).adjoint();

            if (nullSpaceBasisOfA && (computationOptions & ComputeFullV)) {
                //we can compute the null space basis for A
                nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA, nullSpaceRows, nullSpaceCols);
            }
        }
        
        void dampedPseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                 Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                 Eigen::Ref<Eigen::MatrixXd> Apinv,
                                 double tolerance,
                                 double dampingFactor,
                                 unsigned int computationOptions,
                                 double * nullSpaceBasisOfA,
                                 int *nullSpaceRows, int *nullSpaceCols)
        {
            using namespace Eigen;
            
            if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
            svdDecomposition.compute(A, computationOptions);
            
            JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
            
            //rank will be used for the null space basis.
            //not sure if this is correct
            int rank = 0;
            for (int idx = 0; idx < singularValues.size(); idx++) {
                if (tolerance > 0 && singularValues(idx) > tolerance) {
                    rank++;
                }
                singularValues(idx) = singularValues(idx) / ((singularValues(idx) * singularValues(idx)) + (dampingFactor * dampingFactor));
            }

            Apinv = svdDecomposition.matrixV().leftCols(rank) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(rank).adjoint();
            
            if (nullSpaceBasisOfA && nullSpaceRows && nullSpaceCols
                && (computationOptions & ComputeFullV)) {
                //we can compute the null space basis for A
                nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA, *nullSpaceRows, *nullSpaceCols);
            }
        }

        void nullSpaceBasisFromDecomposition(Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                                 double tolerance,
                                                 double * nullSpaceBasisMatrix,
                                                 int &rows, int &cols)
        {
            using namespace Eigen;
            JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
            int rank = 0;
            for (int idx = 0; idx < singularValues.size(); idx++) {
                if (tolerance > 0 && singularValues(idx) > tolerance) {
                    rank++;
                }
            }
            nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisMatrix, rows, cols);
            
        }
        
        void nullSpaceBasisFromDecomposition(Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                             int rank,
                                             double * nullSpaceBasisMatrix,
                                             int &rows, int &cols)
        {
            using namespace Eigen;
            const MatrixXd &vMatrix = svdDecomposition.matrixV();
            //A \in \mathbb{R}^{uMatrix.rows() \times vMatrix.cols()}
            rows = vMatrix.cols();
            cols = vMatrix.cols() - rank;
            Map<MatrixXd> map(nullSpaceBasisMatrix, rows, cols);
            map = vMatrix.rightCols(vMatrix.cols() - rank);
        }

        
        void skewSymmentricMatrixFrom3DVector(const Eigen::Ref<const Eigen::Vector3d>& vector, 
                                              Eigen::Ref<Eigen::Matrix3d> skewSymmetricMatrix)
        {
            skewSymmetricMatrix.setZero();
            //            S = [   0,   -w(3),    w(2);
            //                 w(3),   0,     -w(1);
            //                 -w(2),  w(1),     0   ];
            skewSymmetricMatrix(0, 1) = -vector(2);
            skewSymmetricMatrix(0, 2) = vector(1);
            skewSymmetricMatrix(1, 2) = -vector(0);
            skewSymmetricMatrix.bottomLeftCorner<2, 2>() = -skewSymmetricMatrix.topRightCorner<2, 2>().transpose();
        }
    }
}
