#ifndef HPPCOMMON_H
#define HPPCOMMON_H

#include <cstddef>

#define MEMBER_PTR(type, var)                   \
    protected:                                  \
    type var##_;                                \
public:                                         \
type& var() { return var##_; }                   \
type var() const { return var##_; }             \
void set_##var(type val) { var##_ = val; }      

#define MEMBER_VAR(type, var)                   \
    protected:                                  \
    type var##_;                                \
public:                                         \
type& var() { return var##_; }                  \
const type& var() const { return var##_; }      \
void set_##var(type val) { var##_ = val; }       


#define MEMBER_INIT_NULL(var) var##_(0)
#define MEMBER_INIT_ARG(var) var##_(_##var)
#define MEMBER_INIT(var, val) var##_(val)

#define MEMBER_RELEASE_PTR(var) do { if (var##_) { delete var##_; var##_ = NULL;}  } while(0)



#define CEV2D const Eigen::Vector2d
#define CEV3D const Eigen::Vector3d
#define CEV4D const Eigen::Vector4d
#define CEVXD const Eigen::VectorXd

#define EIGEN_V_VEC2D std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
#define EIGEN_V_VEC3D std::vector< Eigen::Vector3d >
#define EIGEN_V_VEC4D std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >

#define EIGEN_V_MAT2D std::vector< Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >
#define EIGEN_V_MAT3D std::vector< Eigen::Matrix3d >
#define EIGEN_V_MAT4D std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >

#define EIGEN_V_QUATD std::vector< Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond > >


#endif // #ifndef HPPCOMMON_H

