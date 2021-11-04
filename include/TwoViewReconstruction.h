/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TwoViewReconstruction_H
#define TwoViewReconstruction_H

#include<opencv2/opencv.hpp>
#include <memory>
#include <unordered_set>

namespace ORB_SLAM3 {

class TwoViewReconstruction {
  typedef std::pair<int, int> Match;

 public:

  // Fix the reference frame
  TwoViewReconstruction(cv::Mat &k, float sigma = 1.0, int iterations = 200);

  // Computes in parallel a fundamental matrix and a homography
  // Selects a model and tries to recover the motion and the structure from motion
  bool Reconstruct(const std::vector<cv::KeyPoint> &vKeys1,
                   const std::vector<cv::KeyPoint> &vKeys2,
                   const std::vector<int> &vMatches12,
                   cv::Mat &R21,
                   cv::Mat &t21,
                   std::vector<cv::Point3f> &vP3D,
                   std::vector<bool> &vbTriangulated);

 private:
  // PIMPL
  class Impl;
  std::shared_ptr<Impl> impl_;

};

} //namespace ORB_SLAM

#endif // TwoViewReconstruction_H
