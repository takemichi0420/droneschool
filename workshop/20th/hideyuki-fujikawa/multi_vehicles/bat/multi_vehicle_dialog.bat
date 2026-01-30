@echo off

REM ・事前にMission Plannerで対象機体（Copter、Rover）のシミュレータを起動しておくこと
REM ・rover.parm をコピーしてboat.parm を作成し、ファイル末尾に "FRAME_CLASS 2" を追記して保存すること

REM === Mission Planner SITL path selection ===
echo ========================================
echo   Mission Planner SITL パス選択
echo ========================================
echo.
echo 1. ローカルPC（%USERPROFILE%\Documents\Mission Planner\sitl）
echo 2. OneDrive（%OneDrive%\ドキュメント\Mission Planner\sitl）
echo.
set /p CHOICE="選択してください (1 or 2): "

if "%CHOICE%"=="1" (
    SET SITL=%USERPROFILE%\Documents\Mission Planner\sitl
    echo.
    echo ローカルPCのパスを使用します
) else if "%CHOICE%"=="2" (
    SET SITL=%OneDrive%\ドキュメント\Mission Planner\sitl
    echo.
    echo OneDriveのパスを使用します
) else (
    echo.
    echo 無効な選択です。デフォルト（OneDrive）を使用します
    SET SITL=%OneDrive%\ドキュメント\Mission Planner\sitl
)

SET PARAMS=%SITL%\default_params
echo.
echo SITL パス: %SITL%
echo PARAMS パス: %PARAMS%
echo.

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
