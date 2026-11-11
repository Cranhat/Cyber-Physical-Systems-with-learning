@echo off
rem   computer\scripts\ros.cmd            start
rem   computer\scripts\ros.cmd move 0.2   (2nd terminal)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0ros.ps1" %*
