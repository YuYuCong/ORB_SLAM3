#!/bin/bash



# pathDatasetEuRoc='/Datasets/EuRoC' #Example, it is necessary to change it by the dataset path

# # Single Session Example (Pure visual)
# echo "Launching MH01 with Stereo sensor"
# ./Stereo/stereo_euroc ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml "$pathDatasetEuRoc"/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo
# echo "------------------------------------"
# echo "Evaluation of MH01 trajectory with Stereo sensor"
# python ../evaluation/evaluate_ate_scale.py ../evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt f_dataset-MH01_stereo.txt --plot MH01_stereo.pdf



# # MultiSession Example (Pure visual)
# echo "Launching Machine Hall with Stereo sensor"
# ./Stereo/stereo_euroc ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml "$pathDatasetEuRoc"/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuRoc"/MH02 ./Stereo/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuRoc"/MH03 ./Stereo/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuRoc"/MH04 ./Stereo/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuRoc"/MH05 ./Stereo/EuRoC_TimeStamps/MH05.txt dataset-MH01_to_MH05_stereo
# echo "------------------------------------"
# echo "Evaluation of MAchine Hall trajectory with Stereo sensor"
# python ../evaluation/evaluate_ate_scale.py ../evaluation/Ground_truth/EuRoC_left_cam/MH_GT.txt f_dataset-MH01_to_MH05_stereo.txt --plot MH01_to_MH05_stereo.pdf


# # Single Session Example (Visual-Inertial)
# echo "Launching V102 with Monocular-Inertial sensor"
# ./Monocular-Inertial/mono_inertial_euroc ../Vocabulary/ORBvoc.txt ./Monocular-Inertial/EuRoC.yaml "$pathDatasetEuRoc"/V102 ./Monocular-Inertial/EuRoC_TimeStamps/V102.txt dataset-V102_monoi
# echo "------------------------------------"
# echo "Evaluation of V102 trajectory with Monocular-Inertial sensor"
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetEuRoc"/V102/mav0/state_groundtruth_estimate0/data.csv f_dataset-V102_monoi.txt --plot V102_monoi.pdf


# # MultiSession Monocular Examples

# echo "Launching Vicon Room 2 with Monocular-Inertial sensor"
# ./Monocular-Inertial/mono_inertial_euroc ../Vocabulary/ORBvoc.txt ./Monocular-Inertial/EuRoC.yaml "$pathDatasetEuRoc"/V201 ./Monocular-Inertial/EuRoC_TimeStamps/V201.txt "$pathDatasetEuRoc"/V202 ./Monocular-Inertial/EuRoC_TimeStamps/V202.txt "$pathDatasetEuRoc"/V203 ./Monocular-Inertial/EuRoC_TimeStamps/V203.txt dataset-V201_to_V203_monoi
# echo "------------------------------------"
# echo "Evaluation of Vicon Room 2 trajectory with Stereo sensor"
# python ../evaluation/evaluate_ate_scale.py ../evaluation/Ground_truth/EuRoC_imu/V2_GT.txt f_dataset-V201_to_V203_monoi.txt --plot V201_to_V203_monoi.pdf




pathDatasetEuRoc='/Datasets/EuRoc'

DatasetList=(\
"MH_03_medium" \
"MH_04_difficult" \
"MH_05_difficult" \
"V2_01_easy" \
"V2_02_medium" \
"V2_03_difficult"
)

for Dataset in "${DatasetList[@]}"
do
  echo "------------------------------------"
  echo "Launching ""$pathDatasetEuRoc"/"$Dataset"" with Monocular-Inertial sensor"
  ./Monocular-Inertial/mono_inertial_euroc ../Vocabulary/ORBvoc.txt \
  ./Monocular-Inertial/EuRoC.yaml \
  "$pathDatasetEuRoc"/"$Dataset" \
  ./Monocular-Inertial/EuRoC_TimeStamps/"$Dataset".txt \
  "$Dataset"_mono_inertial

  mkdir ORB_SLAM3-results
  mv ./*"$Dataset"_mono_inertial.txt ./ORB_SLAM3-results/
  echo "------------------------------------"
  echo "Evaluation of ""$Dataset"" trajectory with Monocular-Inertial sensor"
  python ../evaluation/evaluate_ate_scale.py \
  "$pathDatasetEuRoc"/"$Dataset"/mav0/state_groundtruth_estimate0/data.csv \
  ./ORB_SLAM3-results/f_"$Dataset"_mono_inertial.txt \
  --plot ./ORB_SLAM3-results/"$Dataset"_mono_inertial.pdf \
  --verbose2
  echo "result can be found in ./ORB_SLAM3-results/"
  echo "------------------------------------"
done