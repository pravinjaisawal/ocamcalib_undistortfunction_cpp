#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h> 

struct ExtrinsicCostFunctor {
    cv::Point3d P; // Checkerboard point in local coordinate(board coordinate)
    cv::Point3d m; // Unit vector from cam2world

    ExtrinsicCostFunctor(const cv::Point3d &P, const cv::Point3d &m) : P(P), m(m) {}

    template <typename T>
        bool operator()(const T* const rotation, const T* const translation, T* residual) const {
        // Create a 3D point array in T
        T point[3] = {T(P.x), T(P.y), T(P.z)};
        
        // Resulting rotated point
        T p[3];

        // Rotate the point
        ceres::AngleAxisRotatePoint(rotation, point, p);

        // Apply translation
        p[0] += translation[0];
        p[1] += translation[1];
        p[2] += translation[2];

        // Normalize to unit vector
        T norm = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        T u[3] = {p[0] / norm, p[1] / norm, p[2] / norm};

        // Compute residual (difference from observed unit vector)
        residual[0] = u[0] - T(m.x);
        residual[1] = u[1] - T(m.y);
        residual[2] = u[2] - T(m.z);

        return true;
    }
};

void estimateExtrinsics(const std::vector<cv::Point3d>& boardPoints,
                        const std::vector<cv::Point3d>& unitVectors,
                        cv::Mat& rvec, cv::Mat& tvec); 
