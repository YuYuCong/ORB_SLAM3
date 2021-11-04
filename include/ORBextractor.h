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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <memory>

namespace ORB_SLAM3 {

class ExtractorNode {
 public:
  ExtractorNode() : bNoMore(false) {}

  void DivideNode(ExtractorNode &n1,
                  ExtractorNode &n2,
                  ExtractorNode &n3,
                  ExtractorNode &n4);

  std::vector<cv::KeyPoint> vKeys;
  cv::Point2i UL, UR, BL, BR;
  std::list<ExtractorNode>::iterator lit;
  bool bNoMore;
};

class ORBextractor {
 public:

  enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

  ORBextractor(int nfeatures, float scaleFactor, int nlevels,
               int iniThFAST, int minThFAST);

  ~ORBextractor() {}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  int operator()(cv::InputArray _image, cv::InputArray _mask,
                 std::vector<cv::KeyPoint> &_keypoints,
                 cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

  std::shared_ptr<const std::vector<cv::Mat>> GetImagePyramid();

  int GetLevels();

  float GetScaleFactor();

  std::vector<float> GetScaleFactors();

  std::vector<float> GetInverseScaleFactors();

  std::vector<float> GetScaleSigmaSquares();

  std::vector<float> GetInverseScaleSigmaSquares();


 private:
   // PIMPL
   class Impl;
   std::shared_ptr<Impl> impl_;
};

} //namespace ORB_SLAM

#endif

