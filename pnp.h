

#pragma once

/* ********************************* FILE ************************************/
/** \file    pnp.h
 *
 * \brief    This header contains the simplest perspective n point solver I can make.
 *
 * Ideally the user should supply the size of the image,
 * the linear intrinsics, as well as the other parameters in the PnpParams dict.
 *
 *
 * \remark
 * - c++11
 * - depends on ceres solver
 * - header only
 * - tested by test_pose.cpp
 *
 * \author   Mikael Persson
 * \date     2015-04-01
 * \note GPL
 *
 ******************************************************************************/

#include "utils/cvl/pose.h"


namespace cvl{









/**
 * @brief The PNPParams class
 * common PNP ransac parameters
 * the pnp below this class is what you want..
 */
class PnpParams {
public:
    PnpParams(){}
    /**
     * @brief PnpParams
     * @param K The Linear calibration matrix
     * @param pixel_threshold the threshold in pixels
     * @param rows the number of image rows
     * @param cols the number of image cols
     */
    PnpParams(cvl::Matrix<double,3,3> K,
              double pixel_threshold, int rows, int cols):
        K(K),pixel_threshold(pixel_threshold/K(0,0)),rows(rows),cols(cols) {}
    // general useful stuff
    /// image rows
    int rows;
    /// image cols
    int cols;
    /// the linear intrinsics of the camera
    cvl::Matrix<double,3,3> K;

    //  parameters
    /// initial inlier ratio estimate, good value = expected inlier ratio/2
    double inlier_ratio=0.25;
    /// minimum probability for never findning a single full inlier set
    double min_probability=0.99999;
    /// pixeldist/focallength
    double threshold=0.001;

    /// perform maximum likelihood sampling consensus,
    bool MLESAC=false;
    /// break early in the ransac loop if a sufficiently good solution is found
    bool early_exit=false;

    /// if random eval is used, this is the number of samples drawn
    int max_support_eval=500;
    int max_min_case_failures=250;
    /// a maximum number of iterations to perform to limit the amount of time spent
    int max_iterations=1000;
    /// the minimum number of iterations to perform to ensure the required number of iterations is correctly computed
    int min_iterations=500;
    /// the total number of iterations performed, this is essentially a output
    int totaliters=0;
};



/**
 * @brief pnp_ransac
 * @param xs 3d points in world
 * @param yns pinhole camera coordinates
 * @param params
 * @return the pose Pcw
 *
 * This is the function you want
 *
 */
PoseD pnp_ransac(const std::vector<cvl::Vector3d>& xs,
                 const std::vector<cvl::Vector2d>& yns,
                 PnpParams params=PnpParams());

/**
 * @brief pnp
 * @param xs 3d points
 * @param yns pinhole camera coordinates
 * @param init
 * @param check_inliers
 * @return the pose
 */
PoseD pnp(const std::vector<cvl::Vector3d>& xs,
          const std::vector<cvl::Vector2d>& yns,
          const PoseD& init,
          bool check_inliers=true);





}// end namespace cvl

