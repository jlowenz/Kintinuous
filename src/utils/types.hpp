#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

typedef float matrix_el_t;
const auto storage_t = Eigen::RowMajor;


// Vector types
typedef Eigen::Matrix<matrix_el_t,2,1> Vector2_t;
typedef Eigen::Matrix<matrix_el_t,3,1> Vector3_t;
typedef Eigen::Matrix<matrix_el_t,4,1> Vector4_t;
typedef Eigen::Matrix<matrix_el_t,6,1> Vector6_t;
typedef Eigen::VectorXf VectorX_t;

typedef Eigen::Matrix<double,2,1> Vector2d_t;
typedef Eigen::Matrix<double,3,1> Vector3d_t;
typedef Eigen::Matrix<double,4,1> Vector4d_t;
typedef Eigen::Matrix<double,6,1> Vector6d_t;
typedef Eigen::VectorXd VectorXd_t;

typedef Eigen::Quaternionf Quaternion_t;

// Matrix types
typedef Eigen::Matrix<matrix_el_t,3,3,storage_t> Matrix3_t;
typedef Eigen::Matrix<matrix_el_t,4,4,storage_t> Matrix4_t;
typedef Eigen::Matrix<matrix_el_t,6,6,storage_t> Matrix6_t;

typedef Eigen::Matrix<double,3,3,storage_t> Matrix3d_t;
typedef Eigen::Matrix<double,4,4,storage_t> Matrix4d_t;
typedef Eigen::Matrix<double,6,6,storage_t> Matrix6d_t;
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,storage_t> MatrixXd_t;

// Collection types
typedef std::vector<Vector2_t, Eigen::aligned_allocator<Vector2_t>> vectors2_t;
typedef std::vector<Vector3_t, Eigen::aligned_allocator<Vector3_t>> vectors3_t;
typedef std::vector<Vector4_t, Eigen::aligned_allocator<Vector4_t>> vectors4_t;

typedef std::vector<Vector3d_t, Eigen::aligned_allocator<Vector3d_t>> vectors3d_t;

typedef std::vector<Matrix3_t, Eigen::aligned_allocator<Matrix3_t>> matrices3_t;
typedef std::vector<Matrix4_t, Eigen::aligned_allocator<Matrix4_t>> matrices4_t;
typedef std::vector<Matrix6_t, Eigen::aligned_allocator<Matrix6_t>> matrices6_t;


