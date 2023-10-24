/*
 * @Author: ymhandcht 898163880@qq.com
 * @Date: 2023-10-19 19:23:44
 * @LastEditors: ymhandcht 898163880@qq.com
 * @LastEditTime: 2023-10-22 22:28:56
 * @FilePath: /test_ws1/src/reprojection/src/SnavelyReprojectionError.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include <iostream>
#include "ceres/ceres.h"
#include "rotation.h"

class SnavelyReprojectionError {
public:
    SnavelyReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
                                                                           observed_y(observation_y) {}

    template<typename T>
    bool operator()(const T *const camera,
                    const T *const point,
                    T *residuals) const {
        // camera[0,1,2] are the angle-axis rotation
        T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - T(observed_x);
        residuals[1] = predictions[1] - T(observed_y);

        return true;
    }

    // camera : 9 dims array
    // [0-2] : angle-axis rotation
    // [3-5] : translateion
    // [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
    // point : 3D location.
    // predictions : 2D predictions with center of the image plane.
    template<typename T>
    //计算投影方程 观测方程
    static inline bool CamProjectionWithDistortion(const T *camera, const T *point, T *predictions) {
        // Rodrigues' formula
        T p[3];  
        AngleAxisRotatePoint(camera, point, p); //p点是点point经过旋转后的点
        // camera[3,4,5] are the translation
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center fo distortion
        //归一化坐标 由于z是负数所以乘以-1
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion
        const T &l1 = camera[7];
        const T &l2 = camera[8];

        T r2 = xp * xp + yp * yp;
        T distortion = T(1.0) + r2 * (l1 + l2 * r2);

        const T &focal = camera[6];
        //经过畸变后的像素坐标
        predictions[0] = focal * distortion * xp;
        predictions[1] = focal * distortion * yp;

        return true;
    }

    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }

private:
    double observed_x;
    double observed_y;
};

#endif // SnavelyReprojection.h

