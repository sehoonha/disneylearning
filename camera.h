#ifndef CAMERA_H
#define CAMERA_H

#include <string>
#include <Eigen/Dense>

namespace disneysimple {
    namespace gui {
        class Camera {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            public:
            Camera();
        
            void trackball(float p1x, float p1y, float p2x, float p2y);
            void add_quat(float p1x, float p1y, float p2x, float p2y);
            void build_rotmatrix(float m[4][4]);

            Eigen::Vector2d begin;
            Eigen::VectorXd q;
            Eigen::Vector3d pos;
            int follow;

        };

    } // namespace gui
} // namespace disneysimple
    
#endif // #ifndef CAMERA_H
