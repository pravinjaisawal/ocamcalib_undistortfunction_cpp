/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/
/*
I have updated to use opencv version =4.5, normally it should work for other opencv version as well
*/


#include "ocam_functions.h"

// Utility function to skip empty or comment lines
bool is_valid_line(const std::string &line)
{
    return !line.empty() && line[0] != '#';
}

// Helper function to parse a line into a vector of doubles
bool parse_doubles(const std::string &line, std::vector<double> &output)
{
    std::stringstream ss(line);
    double value;
    while (ss >> value) {
        output.push_back(value);
    }
    return !output.empty();
}


int get_ocam_model(ocam_model &myocam_model, const std::string &filename)
{
     std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return -1;
    }

    std::string line;
    
    // Read polynomial coefficients for cam2world
    while (std::getline(file, line)) {
        if (is_valid_line(line)) {
            std::stringstream ss(line);
            ss >> myocam_model.length_pol;
            if (ss.fail()) {
                std::cerr << "Error: Failed to read the polynomial length!" << std::endl;
                return -1;
            }
            myocam_model.pol.resize(myocam_model.length_pol);
            for (int i = 0; i < myocam_model.length_pol; ++i) {
                if (!(ss >> myocam_model.pol[i])) {
                    std::cerr << "Error: Failed to read polynomial coefficient!" << std::endl;
                    return -1;
                }
            }
            break; // Exit after reading the polynomial coefficients
        }
    }

    // Read inverse polynomial coefficients for world2cam
    while (std::getline(file, line)) {
        if (is_valid_line(line)) {
            std::stringstream ss(line);
            ss >> myocam_model.length_invpol;
            if (ss.fail()) {
                std::cerr << "Error: Failed to read the inverse polynomial length!" << std::endl;
                return -1;
            }
            myocam_model.invpol.resize(myocam_model.length_invpol);
            for (int i = 0; i < myocam_model.length_invpol; ++i) {
                if (!(ss >> myocam_model.invpol[i])) {
                    std::cerr << "Error: Failed to read inverse polynomial coefficient!" << std::endl;
                    return -1;
                }
            }
            break; // Exit after reading the inverse polynomial coefficients
        }
    }

    // Read center coordinates (xc, yc)
    while (std::getline(file, line)) {
        if (is_valid_line(line)) {
            std::stringstream ss(line);
            ss >> myocam_model.xc >> myocam_model.yc;
            if (ss.fail()) {
                std::cerr << "Error: Failed to read center coordinates!" << std::endl;
                return -1;
            }
            break; // Exit after reading the center coordinates
        }
    }

    // Read affine parameters (c, d, e)
    while (std::getline(file, line)) {
        if (is_valid_line(line)) {
            std::stringstream ss(line);
            ss >> myocam_model.c >> myocam_model.d >> myocam_model.e;
            if (ss.fail()) {
                std::cerr << "Error: Failed to read affine parameters!" << std::endl;
                return -1;
            }
            break; // Exit after reading the affine parameters
        }
    }

    // Read image size (height, width)
    while (std::getline(file, line)) {
        if (is_valid_line(line)) {
            std::stringstream ss(line);
            ss >> myocam_model.height >> myocam_model.width;
            if (ss.fail()) {
                std::cerr << "Error: Failed to read image size!" << std::endl;
                return -1;
            }
            break; // Exit after reading the image size
        }
    }

    file.close();
    return 0; // Successfully loaded the model
}

//------------------------------------------------------------------------------
void cam2world(cv::Point3d &point3D, const cv::Point2f &point2D, const ocam_model &model)
{
    double xp, yp, r, zp, invdet, invnorm;
    invdet  = 1/(model.c - model.d * model.e);
    std::cout << invdet << std::endl;
    // Calculate xp and yp based on the distortion model
    xp = invdet * ((point2D.x - model.xc) - model.d * (point2D.y - model.yc));
    yp = invdet * (-model.e * (point2D.x - model.xc) + model.c * (point2D.y - model.yc));

    std::cout << point2D << " " << xp << " " << yp << std::endl;

    r = sqrt(xp * xp + yp * yp);
    zp = model.pol[0];
    double r_i = 1;

    for (int i = 1; i < model.length_pol; i++) {
        r_i *= r;
        zp += r_i * model.pol[i];
    }
    //normalize to unit norm
    invnorm = 1.0 / sqrt(xp * xp + yp * yp + zp * zp);
    point3D.x = xp * invnorm;
    point3D.y = yp * invnorm;
    point3D.z = zp * invnorm;
  }

//------------------------------------------------------------------------------
void world2cam(cv::Point2f &point2D, const cv::Point3d &point3D, const ocam_model &myocam_model) {
  const auto &invpol = myocam_model.invpol;
  double xc = myocam_model.xc;
  double yc = myocam_model.yc;
  double c = myocam_model.c;
  double d = myocam_model.d;
  double e = myocam_model.e;
 
  double norm = sqrt(point3D.x * point3D.x + point3D.y * point3D.y);

    if (norm != 0) {
        // Calculate theta and rho
        double invnorm = 1.0 / norm;
        double theta = atan(point3D.z / norm);
        double rho = invpol[0];
        double theta_power = theta; // Current theta^i term

        // Compute rho using the inverse polynomial
        for (int i = 1; i < myocam_model.length_invpol; i++) {
            rho += theta_power * invpol[i];
            theta_power *= theta; // Increment the power of theta
        }

        // Compute distorted x and y coordinates
        double x = point3D.x * invnorm * rho;
        double y = point3D.y * invnorm * rho;

        // Map to pixel coordinates using affine parameters
        point2D.x = x * c + y * d + xc;
        point2D.y = x * e + y + yc;
    } else {
        // If norm is zero, return the image center as the projection
        point2D.x = xc;
        point2D.y = yc;
    }
}

//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT( cv::Mat &mapx, cv::Mat &mapy, const ocam_model &ocam_model, float sf) 
{
    int width = mapx.cols;
    int height = mapx.rows;
    float Nxc = height / 2.0f;
    float Nyc = width / 2.0f;
    float Nz = -width / sf;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cv::Point3d M(j - Nyc, i - Nxc, Nz);
            cv::Point2f m;
            world2cam(m, M, ocam_model);

            mapx.at<float>(i, j) = static_cast<float>(m.x); // m.x corresponds to columns (x)
            mapy.at<float>(i, j) = static_cast<float>(m.y); // m.y corresponds to rows (y)
        }
    }
}

//------------------------------------------------------------------------------
void create_panoramic_undistortion_LUT(cv::Mat &mapx, cv::Mat &mapy, float Rmin, float Rmax, float xc, float yc) {
    int width = mapx.cols;
    int height = mapx.rows;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            float theta = -static_cast<float>(j) / width * 2 * M_PI;
            float rho = Rmax - (Rmax - Rmin) / height * i;

            mapx.at<float>(i, j) = yc + rho * sin(theta);
            mapy.at<float>(i, j) = xc + rho * cos(theta);
        }
    }
}
