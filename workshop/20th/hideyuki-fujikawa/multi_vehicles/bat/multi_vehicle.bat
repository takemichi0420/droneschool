@echo off

REM ・SITLパスはマイドキュメントのパス（OneDrive or ローカルPC上）に応じて変更すること 
REM ・事前にMission Plannerで対象機体（Copter、Rover）のシミュレータを起動しておくこと 
REM ・rover.parm をコピーしてboat.parm を作成し、ファイル末尾に "FRAME_CLASS 2" を追記して保存すること

REM === Mission Planner SITL path ===	
REM SET SITL=%USERPROFILE%\Documents\Mission Planner\sitl
SET SITL=%OneDrive%\ドキュメント\Mission Planner\sitl
SET PARAMS=%SITL%\default_params

REM === Rover (instance 0, sysid 1) ===
start "Rover_0" "%SITL%\ArduRover.exe" ^
  --model rover ^
  --instance 0 ^
  --sysid 1 ^
  --home 35.876991,140.348026,0,0 ^
  --defaults "%PARAMS%\rover.parm"

REM === Boat (instance 1, sysid 2) ===
start "Boat_1" "%SITL%\ArduRover.exe" ^
  --model rover ^
  --instance 1 ^
  --sysid 2 ^
  --home 35.879768,140.348495,0,0 ^
  --defaults "%PARAMS%\boat.parm"

REM === Copter (instance 2, sysid 3) ===
start "Copter_2" "%SITL%\ArduCopter.exe" ^
  --model quad ^
  --instance 2 ^
  --sysid 3 ^
  --home 35.878275,140.338069,0,0 ^
  --defaults "%PARAMS%\copter.parm"

pause
