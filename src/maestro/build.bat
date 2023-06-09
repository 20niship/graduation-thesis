REM REMって書くとコメントになるのばぐってるだろ

@echo off
set ECLIPSE_PATH=C:\eclipse\

REM current directory
set WORKSPACE_PATH=%cd%
set PROJECT_NAME=test_project2

REM etract output to terminal cout while buildin
set CDT_BUILD_VERBOSE=1

set CDT_BUILD_LOG_FILE=%WORKSPACE_PATH%\build.log

REM print workspace path
echo %WORKSPACE_PATH%

cd %ECLIPSE_PATH%
REM eclipsec.exe -nosplash -data %WORKSPACE_PATH% -application org.eclipse.cdt.managedbuilder.core.headlessbuild -importAll %PROJECT_NAME% -build PLATINUM_R

@echo off
set ANT_HOME=C:\path\to\apache-ant-x.x.x
set PATH=%ANT_HOME%\bin;%PATH%
ant -f build.xml build