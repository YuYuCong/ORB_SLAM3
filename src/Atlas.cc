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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3 {

class Atlas::Impl {
 public:
  Impl();
  explicit Impl(int initKFid); // When its initialization the first map is created
  ~Impl();

  void CreateNewMap();
  void ChangeMap(Map *pMap);

  unsigned long int GetLastInitKFid();

  void SetViewer(Viewer *pViewer);

  void AddKeyFrame(KeyFrame *pKF);
  void AddMapPoint(MapPoint *pMP);

  void AddCamera(GeometricCamera *pCam);

  void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
  void InformNewBigChange();
  int GetLastBigChangeIdx();

  long unsigned int MapPointsInMap();
  long unsigned KeyFramesInMap();

  std::vector<KeyFrame *> GetAllKeyFrames();
  std::vector<MapPoint *> GetAllMapPoints();
  std::vector<MapPoint *> GetReferenceMapPoints();

  std::vector<Map *> GetAllMaps();

  int CountMaps();

  void clearMap();

  void clearAtlas();

  Map *GetCurrentMap();

  void SetMapBad(Map *pMap);
  void RemoveBadMaps();

  bool isInertial();
  void SetInertialSensor();
  void SetImuInitialized();
  bool isImuInitialized();

  void SetKeyFrameDatabase(KeyFrameDatabase *pKFDB);
  KeyFrameDatabase *GetKeyFrameDatabase();

  void SetORBVocabulary(ORBVocabulary *pORBVoc);
  ORBVocabulary *GetORBVocabulary();

  long unsigned int GetNumLivedKF();

  long unsigned int GetNumLivedMP();

 private:
  std::set<Map *> mspMaps;
  std::set<Map *> mspBadMaps;
  Map *mpCurrentMap;

  std::vector<GeometricCamera *> mvpCameras;
  std::vector<KannalaBrandt8 *> mvpBackupCamKan;
  std::vector<Pinhole *> mvpBackupCamPin;

  std::mutex mMutexAtlas;

  unsigned long int mnLastInitKFidMap;

  Viewer *mpViewer;
  bool mHasViewer;

  // Class references for the map reconstruction from the save file
  KeyFrameDatabase *mpKeyFrameDB;
  ORBVocabulary *mpORBVocabulary;

}; // class Atlas


Atlas::Impl::Impl() {
  mpCurrentMap = static_cast<Map *>(NULL);
}

Atlas::Impl::Impl(int initKFid) : mnLastInitKFidMap(initKFid), mHasViewer(false) {
  mpCurrentMap = static_cast<Map *>(NULL);
  CreateNewMap();
}

Atlas::Impl::~Impl() {
  for (std::set<Map *>::iterator it = mspMaps.begin(), end = mspMaps.end();
       it != end;) {
    Map *pMi = *it;

    if (pMi) {
      delete pMi;
      pMi = static_cast<Map *>(NULL);

      it = mspMaps.erase(it);
    } else
      ++it;

  }
}

void Atlas::Impl::CreateNewMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  cout << "Creation of new map with id: " << Map::nNextId << endl;
  if (mpCurrentMap) {
    cout << "Exits current map " << endl;
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()
          + 1; //The init KF is the next of current maximum

    mpCurrentMap->SetStoredMap();
    cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

    //if(mHasViewer)
    //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
  }
  cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

  mpCurrentMap = new Map(mnLastInitKFidMap);
  mpCurrentMap->SetCurrentMap();
  mspMaps.insert(mpCurrentMap);
}

void Atlas::Impl::ChangeMap(Map *pMap) {
  unique_lock<mutex> lock(mMutexAtlas);
  cout << "Chage to map with id: " << pMap->GetId() << endl;
  if (mpCurrentMap) {
    mpCurrentMap->SetStoredMap();
  }

  mpCurrentMap = pMap;
  mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::Impl::GetLastInitKFid() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mnLastInitKFidMap;
}

void Atlas::Impl::SetViewer(Viewer *pViewer) {
  mpViewer = pViewer;
  mHasViewer = true;
}

void Atlas::Impl::AddKeyFrame(KeyFrame *pKF) {
  Map *pMapKF = pKF->GetMap();
  pMapKF->AddKeyFrame(pKF);
}

void Atlas::Impl::AddMapPoint(MapPoint *pMP) {
  Map *pMapMP = pMP->GetMap();
  pMapMP->AddMapPoint(pMP);
}

void Atlas::Impl::AddCamera(GeometricCamera *pCam) {
  mvpCameras.push_back(pCam);
}

void Atlas::Impl::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs) {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::Impl::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->InformNewBigChange();
}

int Atlas::Impl::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::Impl::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::Impl::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame *> Atlas::Impl::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint *> Atlas::Impl::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint *> Atlas::Impl::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetReferenceMapPoints();
}

vector<Map *> Atlas::Impl::GetAllMaps() {
  unique_lock<mutex> lock(mMutexAtlas);
  struct compFunctor {
    inline bool operator()(Map *elem1, Map *elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  vector<Map *> vMaps(mspMaps.begin(), mspMaps.end());
  sort(vMaps.begin(), vMaps.end(), compFunctor());
  return vMaps;
}

int Atlas::Impl::CountMaps() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mspMaps.size();
}

void Atlas::Impl::clearMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->clear();
}

void Atlas::Impl::clearAtlas() {
  unique_lock<mutex> lock(mMutexAtlas);
  /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
  {
      (*it)->clear();
      delete *it;
  }*/
  mspMaps.clear();
  mpCurrentMap = static_cast<Map *>(NULL);
  mnLastInitKFidMap = 0;
}

Map *Atlas::Impl::GetCurrentMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  if (!mpCurrentMap)
    CreateNewMap();
  while (mpCurrentMap->IsBad())
    usleep(3000);

  return mpCurrentMap;
}

void Atlas::Impl::SetMapBad(Map *pMap) {
  mspMaps.erase(pMap);
  pMap->SetBad();

  mspBadMaps.insert(pMap);
}

void Atlas::Impl::RemoveBadMaps() {
  /*for(Map* pMap : mspBadMaps)
  {
      delete pMap;
      pMap = static_cast<Map*>(NULL);
  }*/
  mspBadMaps.clear();
}

bool Atlas::Impl::isInertial() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->IsInertial();
}

void Atlas::Impl::SetInertialSensor() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetInertialSensor();
}

void Atlas::Impl::SetImuInitialized() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetImuInitialized();
}

bool Atlas::Impl::isImuInitialized() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->isImuInitialized();
}

void Atlas::Impl::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB) {
  mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase *Atlas::Impl::GetKeyFrameDatabase() {
  return mpKeyFrameDB;
}

void Atlas::Impl::SetORBVocabulary(ORBVocabulary *pORBVoc) {
  mpORBVocabulary = pORBVoc;
}

ORBVocabulary *Atlas::Impl::GetORBVocabulary() {
  return mpORBVocabulary;
}

long unsigned int Atlas::Impl::GetNumLivedKF() {
  unique_lock<mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (Map *mMAPi: mspMaps) {
    num += mMAPi->GetAllKeyFrames().size();
  }

  return num;
}

long unsigned int Atlas::Impl::GetNumLivedMP() {
  unique_lock<mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (Map *mMAPi: mspMaps) {
    num += mMAPi->GetAllMapPoints().size();
  }

  return num;
}


Atlas::Atlas():
    impl_(std::make_shared<Impl>()) { }

Atlas::Atlas(int initKFid) : impl_(std::make_shared<Impl>(initKFid)) {}

Atlas::~Atlas() = default;

void Atlas::CreateNewMap(){
  impl_->CreateNewMap();
}

void Atlas::ChangeMap(Map *pMap) {
  impl_->ChangeMap(pMap);
}

unsigned long int Atlas::GetLastInitKFid() {
  return impl_->GetLastInitKFid();
}

void Atlas::SetViewer(Viewer *pViewer) {
  impl_->SetViewer(pViewer);
}

void Atlas::AddKeyFrame(KeyFrame *pKF) {
  impl_->AddKeyFrame(pKF);
}
void Atlas::AddMapPoint(MapPoint *pMP) {
  impl_->AddMapPoint(pMP);
}

void Atlas::AddCamera(GeometricCamera *pCam) {
  impl_->AddCamera(pCam);
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs) {
  impl_->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
  impl_->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
  return impl_->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
  return impl_->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
  return impl_->KeyFramesInMap();
}

std::vector<KeyFrame *> Atlas::GetAllKeyFrames() {
  return impl_->GetAllKeyFrames();
}

std::vector<MapPoint *> Atlas::GetAllMapPoints() {
  return impl_->GetAllMapPoints();
}

std::vector<MapPoint *> Atlas::GetReferenceMapPoints() {
  return impl_->GetReferenceMapPoints();
}

std::vector<Map *> Atlas::GetAllMaps() {
  return impl_->GetAllMaps();
}

int Atlas::CountMaps() {
  return impl_->CountMaps();
}

void Atlas::clearMap() {
  impl_->clearMap();
}

void Atlas::clearAtlas() {
  impl_->clearAtlas();
}

Map *Atlas::GetCurrentMap(){
  return impl_->GetCurrentMap();
}

void Atlas::SetMapBad(Map *pMap) {
  impl_->SetMapBad(pMap);
}

void Atlas::RemoveBadMaps() {
  impl_->RemoveBadMaps();
}

bool Atlas::isInertial() {
  return impl_->isInertial();
}

void Atlas::SetInertialSensor(){
  impl_->SetInertialSensor();
}

void Atlas::SetImuInitialized(){
  impl_->SetImuInitialized();
}

bool Atlas::isImuInitialized(){
  return impl_->isImuInitialized();
}

void Atlas::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB){
  impl_->SetKeyFrameDatabase(pKFDB);
}

KeyFrameDatabase *Atlas::GetKeyFrameDatabase(){
  return impl_->GetKeyFrameDatabase();
}

void Atlas::SetORBVocabulary(ORBVocabulary *pORBVoc){
  impl_->SetORBVocabulary(pORBVoc);
}

ORBVocabulary *Atlas::GetORBVocabulary(){
  return impl_->GetORBVocabulary();
}

long unsigned int Atlas::GetNumLivedKF(){
  return impl_->GetNumLivedKF();
}

long unsigned int Atlas::GetNumLivedMP(){
  return impl_->GetNumLivedMP();
}

} //namespace ORB_SLAM3
