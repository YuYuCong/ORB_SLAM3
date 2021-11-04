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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>

namespace ORB_SLAM3 {

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

class System::Impl {
 public:
  Impl(System *pSys,
       const string &strVocFile,
       const string &strSettingsFile,
       const eSensor sensor,
       const bool bUseViewer = true,
       const int initFr = 0,
       const string &strSequence = std::string(),
       const string &strLoadingFile = std::string());

  cv::Mat TrackStereo(const cv::Mat &imLeft,
                      const cv::Mat &imRight,
                      const double &timestamp,
                      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                      string filename = "");

  cv::Mat TrackRGBD(const cv::Mat &im,
                    const cv::Mat &depthmap,
                    const double &timestamp,
                    string filename = "");

  cv::Mat TrackMonocular(const cv::Mat &im,
                         const double &timestamp,
                         const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                         string filename = "");

  void ActivateLocalizationMode();
  void DeactivateLocalizationMode();

  bool MapChanged();

  void Reset();
  void ResetActiveMap();

  void Shutdown();

  void SaveTrajectoryTUM(const string &filename);

  void SaveKeyFrameTrajectoryTUM(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename);

  void SaveTrajectoryKITTI(const string &filename);

  // TODO: Save/Load functions
  // SaveMap(const string &filename);
  // LoadMap(const string &filename);

  int GetTrackingState();
  std::vector<MapPoint *> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  // For debugging
  double GetTimeFromIMUInit();
  bool isLost();
  bool isFinished();

  void ChangeDataset();

#ifdef REGISTER_TIMES
  void InsertRectTime(double& time);

  void InsertTrackTime(doubl e& time);
#endif

 private:

  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary *mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop detection).
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Atlas structure that stores the pointers to all KeyFrames and MapPoints.
  Atlas *mpAtlas;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints and
  // performs relocalization if tracking fails.
  Tracking *mpTracker;

  // Local Mapper. It manages the local map and performs local bundle adjustment.
  LocalMapping *mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
  // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
  LoopClosing *mpLoopCloser;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  Viewer *mpViewer;

  FrameDrawer *mpFrameDrawer;
  MapDrawer *mpMapDrawer;

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the System object.
  std::thread *mptLocalMapping;
  std::thread *mptLoopClosing;
  std::thread *mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;
  bool mbResetActiveMap;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint *> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;
};

System::Impl::Impl(System *pSys,
                   const string &strVocFile,
                   const string &strSettingsFile,
                   const eSensor sensor,
                   const bool bUseViewer,
                   const int initFr,
                   const string &strSequence,
                   const string &strLoadingFile) :
    mSensor(sensor),
    mpViewer(static_cast<Viewer *>(NULL)),
    mbReset(false),
    mbResetActiveMap(false),
    mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false) {
  // Output welcome message
  cout << endl <<
       "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza."
       << endl <<
       "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza."
       << endl <<
       "This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
       "This is free software, and you are welcome to redistribute it" << endl
       <<
       "under certain conditions. See LICENSE.txt." << endl << endl;

  cout << "Input sensor was set to: ";

  if (mSensor == MONOCULAR)
    cout << "Monocular" << endl;
  else if (mSensor == STEREO)
    cout << "Stereo" << endl;
  else if (mSensor == RGBD)
    cout << "RGB-D" << endl;
  else if (mSensor == IMU_MONOCULAR)
    cout << "Monocular-Inertial" << endl;
  else if (mSensor == IMU_STEREO)
    cout << "Stereo-Inertial" << endl;

  //Check settings file
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
    exit(-1);
  }

  bool loadedAtlas = false;

  //----
  //Load ORB Vocabulary
  cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

  mpVocabulary = new ORBVocabulary();
  bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  if (!bVocLoad) {
    cerr << "Wrong path to vocabulary. " << endl;
    cerr << "Falied to open at: " << strVocFile << endl;
    exit(-1);
  }
  cout << "Vocabulary loaded!" << endl << endl;

  //Create KeyFrame Database
  mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

  //Create the Atlas
  mpAtlas = new Atlas(0);

  if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR)
    mpAtlas->SetInertialSensor();

  //Create Drawers. These are used by the Viewer
  mpFrameDrawer = new FrameDrawer(mpAtlas);
  mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile);

  //Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this constructor)
  cout << "Seq. Name: " << strSequence << endl;
  mpTracker = new Tracking(pSys,
                           mpVocabulary,
                           mpFrameDrawer,
                           mpMapDrawer,
                           mpAtlas,
                           mpKeyFrameDatabase,
                           strSettingsFile,
                           mSensor,
                           strSequence);

  //Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(pSys,
                                   mpAtlas,
                                   mSensor == MONOCULAR
                                       || mSensor == IMU_MONOCULAR,
                                   mSensor == IMU_MONOCULAR
                                       || mSensor == IMU_STEREO,
                                   strSequence);
  mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
  mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
  if (mpLocalMapper->mThFarPoints != 0) {
    cout << "Discard points further than " << mpLocalMapper->mThFarPoints
         << " m from current camera" << endl;
    mpLocalMapper->mbFarPoints = true;
  } else
    mpLocalMapper->mbFarPoints = false;

  //Initialize the Loop Closing thread and launch
  mpLoopCloser = new LoopClosing(mpAtlas,
                                 mpKeyFrameDatabase,
                                 mpVocabulary,
                                 mSensor != MONOCULAR); // mSensor!=MONOCULAR);
  mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

  //Initialize the Viewer thread and launch
  if (bUseViewer) {
    mpViewer = new Viewer(pSys,
                          mpFrameDrawer,
                          mpMapDrawer,
                          mpTracker,
                          strSettingsFile);
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);
    mpLoopCloser->mpViewer = mpViewer;
    mpViewer->both = mpFrameDrawer->both;
  }

  //Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  // Fix verbosity
  Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

cv::Mat System::Impl::TrackStereo(const cv::Mat &imLeft,
                                  const cv::Mat &imRight,
                                  const double &timestamp,
                                  const vector<IMU::Point> &vImuMeas,
                                  string filename) {
  if (mSensor != STEREO && mSensor != IMU_STEREO) {
    cerr
        << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial."
        << endl;
    exit(-1);
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      cout << "Reset stereo..." << endl;
      mbReset = false;
      mbResetActiveMap = false;
    } else if (mbResetActiveMap) {
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
    }
  }

  if (mSensor == System::IMU_STEREO)
    for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
      mpTracker->GrabImuData(vImuMeas[i_imu]);

  cv::Mat
      Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp, filename);

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

  return Tcw;
}

cv::Mat System::Impl::TrackRGBD(const cv::Mat &im,
                                const cv::Mat &depthmap,
                                const double &timestamp,
                                string filename) {
  if (mSensor != RGBD) {
    cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD."
         << endl;
    exit(-1);
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
      mbResetActiveMap = false;
    } else if (mbResetActiveMap) {
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
    }
  }

  cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp, filename);

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
  return Tcw;
}

cv::Mat System::Impl::TrackMonocular(const cv::Mat &im,
                                     const double &timestamp,
                                     const vector<IMU::Point> &vImuMeas,
                                     string filename) {
  if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR) {
    cerr
        << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial."
        << endl;
    exit(-1);
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
      mbResetActiveMap = false;
    } else if (mbResetActiveMap) {
      cout << "SYSTEM-> Reseting active map in monocular case" << endl;
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
    }
  }

  if (mSensor == System::IMU_MONOCULAR)
    for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
      mpTracker->GrabImuData(vImuMeas[i_imu]);

  cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp, filename);

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

  return Tcw;
}

void System::Impl::ActivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::Impl::DeactivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

bool System::Impl::MapChanged() {
  static int n = 0;
  int curn = mpAtlas->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else
    return false;
}

void System::Impl::Reset() {
  unique_lock<mutex> lock(mMutexReset);
  mbReset = true;
}

void System::Impl::ResetActiveMap() {
  unique_lock<mutex> lock(mMutexReset);
  mbResetActiveMap = true;
}

void System::Impl::Shutdown() {
  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();
  if (mpViewer) {
    mpViewer->RequestFinish();
    while (!mpViewer->isFinished())
      usleep(5000);
  }

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()
      || mpLoopCloser->isRunningGBA()) {
    if (!mpLocalMapper->isFinished())
      cout << "mpLocalMapper is not finished" << endl;
    if (!mpLoopCloser->isFinished())
      cout << "mpLoopCloser is not finished" << endl;
    if (mpLoopCloser->isRunningGBA()) {
      cout << "mpLoopCloser is running GBA" << endl;
      cout << "break anyway..." << endl;
      break;
    }
    usleep(5000);
  }

  if (mpViewer)
    pangolin::BindToContext("ORB-SLAM2: Map Viewer");

#ifdef REGISTER_TIMES
  mpTracker->PrintTimeStats();
#endif
}

void System::Impl::SaveTrajectoryTUM(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (mSensor == MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
    return;
  }

  vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end(); lit != lend;
       lit++, lRit++, lT++, lbL++) {
    if (*lbL)
      continue;

    KeyFrame *pKF = *lRit;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    cv::Mat Tcw = (*lit) * Trw;
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    vector<float> q = Converter::toQuaternion(Rwc);

    f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0)
      << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0]
      << " " << q[1] << " " << q[2] << " " << q[3] << endl;
  }
  f.close();
  // cout << endl << "trajectory saved!" << endl;
}

void System::Impl::SaveKeyFrameTrajectoryTUM(const string &filename) {
  cout << endl << "Saving keyframe trajectory to " << filename << " ..."
       << endl;

  vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (pKF->isBad())
      continue;

    cv::Mat R = pKF->GetRotation().t();
    vector<float> q = Converter::toQuaternion(R);
    cv::Mat t = pKF->GetCameraCenter();
    f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " "
      << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
      << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

  }

  f.close();
}

void System::Impl::SaveTrajectoryEuRoC(const string &filename) {

  cout << endl << "Saving trajectory to " << filename << " ..." << endl;
  /*if(mSensor==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
      return;
  }*/

  vector<Map *> vpMaps = mpAtlas->GetAllMaps();
  Map *pBiggerMap;
  int numMaxKFs = 0;
  for (Map *pMap: vpMaps) {
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
  if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
    Twb = vpKFs[0]->GetImuPose();
  else
    Twb = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();

  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end(); lit != lend;
       lit++, lRit++, lT++, lbL++) {
    if (*lbL)
      continue;

    KeyFrame *pKF = *lRit;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    if (!pKF)
      continue;

    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    if (!pKF || pKF->GetMap() != pBiggerMap) {
      continue;
    }

    Trw = Trw * pKF->GetPose()
        * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO) {
      cv::Mat Tbw = pKF->mImuCalib.Tbc * (*lit) * Trw;
      cv::Mat Rwb = Tbw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat twb = -Rwb * Tbw.rowRange(0, 3).col(3);
      vector<float> q = Converter::toQuaternion(Rwb);
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
        << twb.at<float>(0) << " " << twb.at<float>(1) << " "
        << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " "
        << q[3] << endl;
    } else {
      cv::Mat Tcw = (*lit) * Trw;
      cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
      vector<float> q = Converter::toQuaternion(Rwc);
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
        << twc.at<float>(0) << " " << twc.at<float>(1) << " "
        << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " "
        << q[3] << endl;
    }

  }
  //cout << "end saving trajectory" << endl;
  f.close();
  cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::Impl::SaveKeyFrameTrajectoryEuRoC(const string &filename) {
  cout << endl << "Saving keyframe trajectory to " << filename << " ..."
       << endl;

  vector<Map *> vpMaps = mpAtlas->GetAllMaps();
  Map *pBiggerMap;
  int numMaxKFs = 0;
  for (Map *pMap: vpMaps) {
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    if (pKF->isBad())
      continue;
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO) {
      cv::Mat R = pKF->GetImuRotation().t();
      vector<float> q = Converter::toQuaternion(R);
      cv::Mat twb = pKF->GetImuPosition();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb.at<float>(0) << " " << twb.at<float>(1) << " "
        << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " "
        << q[3] << endl;

    } else {
      cv::Mat R = pKF->GetRotation();
      vector<float> q = Converter::toQuaternion(R);
      cv::Mat t = pKF->GetCameraCenter();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
  }
  f.close();
}

void System::Impl::SaveTrajectoryKITTI(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (mSensor == MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
    return;
  }

  vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end(); lit != lend;
       lit++, lRit++, lT++) {
    ORB_SLAM3::KeyFrame *pKF = *lRit;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    cv::Mat Tcw = (*lit) * Trw;
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1)
      << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " <<
      Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " "
      << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " <<
      Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " "
      << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
  }
  f.close();
}

int System::Impl::GetTrackingState() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackingState;
}

vector<MapPoint *> System::Impl::GetTrackedMapPoints() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::Impl::GetTrackedKeyPointsUn() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedKeyPointsUn;
}

double System::Impl::GetTimeFromIMUInit() {
  double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  if ((aux > 0.) && mpAtlas->isImuInitialized())
    return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  else
    return 0.f;
}

bool System::Impl::isLost() {
  if (!mpAtlas->isImuInitialized())
    return false;
  else {
    if (mpTracker->mState == Tracking::LOST)
      return true;
    else
      return false;
  }
}

bool System::Impl::isFinished() {
  return (GetTimeFromIMUInit() > 0.1);
}

void System::Impl::ChangeDataset() {
  if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12) {
    mpTracker->ResetActiveMap();
  } else {
    mpTracker->CreateMapInAtlas();
  }

  mpTracker->NewDataset();
}

#ifdef REGISTER_TIMES
void System::Impl::InsertRectTime(double& time) {
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::Impl::InsertTrackTime(double& time) {
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

/*** Implementation of System ***/
System::System(const string &strVocFile,
               const string &strSettingsFile,
               const eSensor sensor,
               const bool bUseViewer,
               const int initFr,
               const string &strSequence,
               const string &strLoadingFile) :
    impl_(std::make_shared<Impl>(this,
                                 strVocFile,
                                 strSettingsFile,
                                 sensor,
                                 bUseViewer,
                                 initFr,
                                 strSequence,
                                 strLoadingFile)) {
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft,
                            const cv::Mat &imRight,
                            const double &timestamp,
                            const vector<IMU::Point> &vImuMeas,
                            std::string filename) {
  return impl_->TrackStereo(imLeft, imRight, timestamp, vImuMeas, filename);
}

cv::Mat System::TrackRGBD(const cv::Mat &im,
                          const cv::Mat &depthmap,
                          const double &timestamp,
                          std::string filename) {
  return impl_->TrackRGBD(im, depthmap, timestamp, filename);
}

cv::Mat System::TrackMonocular(const cv::Mat &im,
                               const double &timestamp,
                               const vector<IMU::Point> &vImuMeas,
                               std::string filename) {
  return impl_->TrackMonocular(im, timestamp, vImuMeas, filename);
};

void System::ActivateLocalizationMode() {
  impl_->ActivateLocalizationMode();
}

void System::DeactivateLocalizationMode() {
  impl_->DeactivateLocalizationMode();
}

bool System::MapChanged() {
  return impl_->MapChanged();
}

void System::Reset() {
  impl_->Reset();
}

void System::ResetActiveMap() {
  impl_->ResetActiveMap();
}

void System::Shutdown() {
  impl_->Shutdown();
}

void System::SaveTrajectoryTUM(const string &filename) {
  impl_->SaveTrajectoryTUM(filename);
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
  impl_->SaveKeyFrameTrajectoryTUM(filename);
}

void System::SaveTrajectoryEuRoC(const string &filename) {
  impl_->SaveTrajectoryEuRoC(filename);
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename) {
  impl_->SaveKeyFrameTrajectoryEuRoC(filename);
}

void System::SaveTrajectoryKITTI(const string &filename) {
  impl_->SaveTrajectoryKITTI(filename);
}

// TODO: Save/Load functions
// SaveMap(const string &filename);
// LoadMap(const string &filename);

int System::GetTrackingState() {
  return impl_->GetTrackingState();
}

std::vector<MapPoint *> System::GetTrackedMapPoints() {
  return impl_->GetTrackedMapPoints();
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
  return impl_->GetTrackedKeyPointsUn();
}

double System::GetTimeFromIMUInit() {
  return impl_->GetTimeFromIMUInit();
}

bool System::isLost() {
  return impl_->isLost();
}

bool System::isFinished() {
  return impl_->isFinished();
}

void System::ChangeDataset() {
  impl_->ChangeDataset();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time){
  impl_->InsertRectTime(time);
}

void System::InsertTrackTime(doubl e& time){
  impl_->InsertTrackTime(time);
}
#endif

} //namespace ORB_SLAM


