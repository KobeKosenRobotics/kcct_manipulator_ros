#ifndef EC_CALCULATOR_EIGEN_UTILITY_H
#define EC_CALCULATOR_EIGEN_UTILITY_H

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ec_calculator
{
    class EigenUtility
    {
        private:
            EigenUtility() {}
            EigenUtility(const EigenUtility&) = delete;
            EigenUtility &operator=(const EigenUtility&) = delete;
        public:
            ~EigenUtility()
            {

            }

            static EigenUtility &get()
            {
                static EigenUtility eu_;
                return eu_;
            }

            Eigen::Matrix<double, 3, 3> getIdentity3()
            {
                Eigen::Matrix<double, 3, 3> identity_;
                identity_.setIdentity();
                return identity_;
            }

            Eigen::Matrix<double, 4, 4> getIdentity4()
            {
                Eigen::Matrix<double, 4, 4> identity_;
                identity_.setIdentity();
                return identity_;
            }

            Eigen::Matrix<double, -1, -1> getIdentity(const int &dimensions)
            {
                Eigen::Matrix<double, -1, -1> identity_;
                identity_.resize(dimensions, dimensions);
                identity_.setIdentity();
                return identity_;
            }

            Eigen::Matrix<double, -1, -1> getZeros(const int &rows, const int &columns)
            {
                Eigen::Matrix<double, -1, -1> zeros_;
                zeros_.resize(rows, columns);
                zeros_.setZero();
                return zeros_;
            }

            Eigen::Matrix<double, 3, 3> hat(const Eigen::Matrix<double, 3, 1> &vec)
            {
                Eigen::Matrix<double, 3, 3> hat_vec_;
                hat_vec_ <<
                    +0.0,       -vec(2,0),  +vec(1,0),
                    +vec(2,0),  +0.0,       -vec(0,0),
                    -vec(1,0),  +vec(0,0),  +0.0;
                return hat_vec_;
            }


            Eigen::MatrixXd getPseudoInverseMatrix(const Eigen::MatrixXd &mat)
            {
                Eigen::MatrixXd pseudo_inv_mat_;

                Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
                Eigen::MatrixXd U = svd.matrixU();
                Eigen::MatrixXd S = svd.singularValues().asDiagonal();
                Eigen::MatrixXd V = svd.matrixV();

                pseudo_inv_mat_ = V*S.inverse()*U.transpose();

                return pseudo_inv_mat_;
            }

            Eigen::MatrixXd getKernel(const Eigen::MatrixXd &mat_)
            {
                Eigen::MatrixXd kernel_;

                kernel_ = mat_.fullPivLu().kernel();

                return kernel_;
            }

            int getRank(const Eigen::MatrixXd &mat)
            {
                Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(mat);
                int rank_ = lu_decomp.rank();

                return rank_;
            }

            Eigen::Matrix<double, 3, 3> getRotationMatrixX(const double &angle_rad)
            {
                Eigen::Matrix<double, 3,3> rot_mat_x_;
                rot_mat_x_ <<
                    +1.0,   +0.0,               +0.0,
                    +0.0,   +cos(angle_rad),    -sin(angle_rad),
                    +0.0,   +sin(angle_rad),    +cos(angle_rad);
                return rot_mat_x_;
            }

            Eigen::Matrix<double, 3, 3> getRotationMatrixY(const double &angle_rad)
            {
                Eigen::Matrix<double, 3,3> rot_mat_y_;
                rot_mat_y_ <<
                    +cos(angle_rad),    +0.0,   +sin(angle_rad),
                    +0.0,               +1.0,   +0.0,
                    -sin(angle_rad),    +0.0,   +cos(angle_rad);
                return rot_mat_y_;
            }

            Eigen::Matrix<double, 3, 3> getRotationMatrixZ(const double &angle_rad)
            {
                Eigen::Matrix<double, 3,3> rot_mat_z_;
                rot_mat_z_ <<
                    +cos(angle_rad),    -sin(angle_rad),    +0.0,
                    +sin(angle_rad),    +cos(angle_rad),    +0.0,
                    +0.0,               +0.0,               +1.0;
                return rot_mat_z_;
            }

            Eigen::Matrix<double, 6, 1> getPose(const Eigen::Matrix<double, 4, 4> &homogeneous_trans_mat)
            {
                Eigen::Matrix<double, 6, 1> pose_;

                pose_.block(0,0,3,1) = homogeneous_trans_mat.block(0,3,3,1);

                pose_.block(3,0,3,1) = getEulerAngle(homogeneous_trans_mat.block(0,0,3,3));

                return pose_;
            }

            Eigen::Matrix<double, 3, 1> getEulerAngle(const Eigen::Matrix<double, 3, 3> &rotation_mat_)
            {
                Eigen::Matrix<double, 3, 1> euler_angle_;

                euler_angle_(1,0) = -asin(rotation_mat_(2,0));

                euler_angle_(0,0) = +acos(rotation_mat_(0,0)/cos(euler_angle_(1,0)));
                if(rotation_mat_(1,0)/cos(euler_angle_(1,0)) < 0) euler_angle_(0,0) *= (-1);

                euler_angle_(2,0) = acos(rotation_mat_(2,2)/cos(euler_angle_(1,0)));
                if(rotation_mat_(2,1)/cos(euler_angle_(1,0)) < 0) euler_angle_(2,0) *=(-1);

                for(int i = 0; i < 3; i++)
                {
                    if(std::isnan(euler_angle_(i,0))) euler_angle_(i,0) = 0.0;
                }

                return euler_angle_;
            }

            Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix<double, 4, 4> &mat)
            {
                Eigen::Matrix<double, 3, 3> rot_mat_;
                Eigen::Matrix<double, 3, 1> pos_;
                Eigen::Matrix<double, 6, 6> adjoint_mat_;

                rot_mat_ <<
                    mat(0,0), mat(0,1), mat(0,2),
                    mat(1,0), mat(1,1), mat(1,2),
                    mat(2,0), mat(2,1), mat(2,2);

                pos_ <<
                    mat(0,3),
                    mat(1,3),
                    mat(2,3);

                adjoint_mat_ <<
                    rot_mat_,
                    this->hat(pos_) * rot_mat_,
                    Eigen::Matrix<double, 3, 3>::Zero(),
                    rot_mat_;

                return adjoint_mat_;
            }

            Eigen::Matrix<double, 6, 6> adjointInverse(const Eigen::Matrix<double, 4, 4> &mat)
            {
                Eigen::Matrix<double, 3, 3> rot_mat_;
                Eigen::Matrix<double, 3, 1> pos_;
                Eigen::Matrix<double, 6, 6> adjoint_inv_mat_;

                rot_mat_ <<
                    mat(0,0), mat(0,1), mat(0,2),
                    mat(1,0), mat(1,1), mat(1,2),
                    mat(2,0), mat(2,1), mat(2,2);

                pos_ <<
                    mat(0,3),
                    mat(1,3),
                    mat(2,3);

                adjoint_inv_mat_ <<
                    rot_mat_.transpose(),
                    -rot_mat_.transpose() * this->hat(pos_),
                    Eigen::Matrix<double, 3, 3>::Zero(),
                    rot_mat_.transpose();

                return adjoint_inv_mat_;
            }

            Eigen::Matrix<double, 6, 6> getTransformationMatrix(const Eigen::Matrix<double, 4, 4> &homogeneous_trans_mat)
            {
                Eigen::Matrix<double, 6, 6> transformation_mat_;
                transformation_mat_ <<
                    homogeneous_trans_mat.block(0, 0, 3, 3).transpose(), Eigen::Matrix<double, 3, 3>::Zero(),
                    Eigen::Matrix<double, 3, 3>::Zero(), getTransformationEuler(getPose(homogeneous_trans_mat).block(3, 0, 3, 1));

                return transformation_mat_;
            }

            Eigen::Matrix<double, 3, 3> getTransformationEuler(const Eigen::Matrix<double, 3, 1> &euler)
            {
                Eigen::Matrix<double, 3, 3> trans_euler_;
                trans_euler_ <<
                    -sin(euler(1,0)),                   +0.0,               +1.0,
                    +cos(euler(1,0))*sin(euler(2,0)),   +cos(euler(2,0)),   +0.0,
                    +cos(euler(2,0))*cos(euler(1,0)),   -sin(euler(2,0)),   +0.0;

                return trans_euler_;
            }

            Eigen::Matrix<double, -1, 1> array2Matrix(const std::vector<float> &array_)
            {
                Eigen::Matrix<double, -1, 1> matrix_;
                int size_ = array_.size();
                matrix_.resize(size_, 1);

                for(int index = 0; index < size_; index++)
                {
                    matrix_(index, 0) = array_[index];
                }

                return matrix_;
            }

            std::vector<float> matrix2Array(const Eigen::Matrix<double, -1, 1> &matrix_)
            {
                std::vector<float> array_;
                int size_ = matrix_.rows();
                array_.resize(size_);

                for(int index = 0; index < size_; index++)
                {
                    array_[index] = matrix_(index, 0);
                }

                return array_;
            }
    };
}

#define EigenUtility ec_calculator::EigenUtility::get()

#endif