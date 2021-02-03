set SLAM=x64\Release\slam.exe
set pathDatasetTUM_VI=C:/Users/Zaid/Downloads

%SLAM% mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM_512.yaml %pathDatasetTUM_VI%/dataset-corridor1_512_16/mav0/cam0/data Examples/Monocular/TUM_TimeStamps/dataset-corridor1_512.txt dataset-corridor1_512_mono