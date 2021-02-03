set SLAM=x64\Release\slam.exe
set pathDataset= data\frames

%SLAM% rgbd_tum Vocabulary/ORBvoc.txt K4A.yaml %pathDataset% %pathDataset%/associated.txt > data\k4a_room.txt