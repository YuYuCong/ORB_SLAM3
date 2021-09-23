#!/bin/bash
# pathDatasetTUM_VI='/Datasets/TUM_VI' #Example, it is necesary to change it by the dataset path


# # Single Session Example

# echo "Launching Magistrale 1 with Stereo-Inertial sensor"
# ./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt ./Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam1/data ./Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt ./Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt dataset-magistrale1_512_stereoi
# echo "------------------------------------"
# echo "Evaluation of Magistrale 1 trajectory with Stereo-Inertial sensor"
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/magistrale1_512_16/mav0/mocap0/data.csv f_dataset-magistrale1_512_stereoi.txt --plot magistrale1_512_stereoi.pdf





pathDatasetTUM_VI='/Datasets/TUM_VI'

Dataset='dataset-room5_512_16'
echo "------------------------------------"
echo "Launching ""$Dataset"" with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt \
./Monocular-Inertial/TUM_512_far.yaml \
"$pathDatasetTUM_VI"/"$Dataset"/mav0/cam0/data \
./Monocular-Inertial/TUM_TimeStamps/"$Dataset".txt \
 "$pathDatasetTUM_VI"/"$Dataset"/mav0/imu0/data.csv \
"$Dataset"_mono_inertial

mkdir ORB_SLAM3-results
mv ./*"$Dataset"_mono_inertial.txt ./ORB_SLAM3-results/
echo "------------------------------------"
echo "Evaluation of ""$Dataset"" trajectory with Monocular-Inertial sensor"
python ../evaluation/evaluate_ate_scale.py \
"$pathDatasetTUM_VI"/"$Dataset"/mav0/mocap0/data.csv \
./ORB_SLAM3-results/f_"$Dataset"_mono_inertial.txt \
--plot ./ORB_SLAM3-results/"$Dataset"_mono_inertial.pdf \
--verbose2
echo "result can be found in ./ORB_SLAM3-results/"
echo "------------------------------------"
