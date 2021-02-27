call C:\ProgramData\Anaconda3\Scripts\activate.bat C:\ProgramData\Anaconda3
call cd /D "%~dp0"
call conda activate Holoenv
set pathDataset= data

python open3d\create_mesh.py

