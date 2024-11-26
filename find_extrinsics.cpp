#include "find_extrinsics.h"

void estimateExtrinsics(const std::vector<cv::Point3d>& boardPoints,
                        const std::vector<cv::Point3d>& unitVectors,
                        cv::Mat& rvec, cv::Mat& tvec) {
    // Initialize rotation and translation
    double rotation[3] = {0, 0, 0}; // Identity rotation
    double translation[3] = {0, 0, 1}; // Approximate translation along Z

    ceres::Problem problem;
    for (size_t i = 0; i < boardPoints.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ExtrinsicCostFunctor, 3, 3, 3>(
                new ExtrinsicCostFunctor(boardPoints[i], unitVectors[i])),
            nullptr,
            rotation,
            translation);
    }

    // Solve the optimization problem
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Copy results to OpenCV Mat
    rvec = cv::Mat(3, 1, CV_64F, rotation).clone();
    tvec = cv::Mat(3, 1, CV_64F, translation).clone();
}
