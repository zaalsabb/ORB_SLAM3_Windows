call C:\ProgramData\Anaconda3\Scripts\activate.bat C:\ProgramData\Anaconda3
call cd /D "%~dp0"
call conda activate Holoenv
set pathDataset= data

python open3d\sensors\azure_kinect_recorder.py --output %pathDataset% --config open3d\sensors\k4a_record_config.json

