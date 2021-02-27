call C:\ProgramData\Anaconda3\Scripts\activate.bat C:\ProgramData\Anaconda3
call cd /D "%~dp0"
call conda activate Holoenv
set pathDataset=data
set fileMkv=2021-02-05-00-52-41.mkv

python open3d\sensors\azure_kinect_mkv_reader.py --input %pathDataset%\%fileMkv% --output %pathDataset%\frames

