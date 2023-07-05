REM arg1 : filename
@echo off

set TARGET="PLATINUM_R"
set NAME="test_project2.pexe"
set LOCAL_PATH="%TARGET%/%NAME%"

REM scp to remote server
set SERVER="192.168.2.52"
set USER="user"
set PASS="user"
set REMOTE_PATH="/mnt/jffs/usr/%NAME%"

REM "if file not exists show error"
if not exist %LOCAL_PATH% (
    echo "file not exists!!"
    EXIT
)

echo "sending file to %SERVER%  :  ...-> %REMOTE_PATH%"
scp %LOCAL_PATH% %USER%@%SERVER%:%REMOTE_PATH%
echo "upload done!!"

REM sshして書き込み先のファイルを実行できるようにする
echo "ssh to %SERVER% and chmod +x %REMOTE_PATH%"
ssh %USER%@%SERVER% chmod +x %REMOTE_PATH%
echo "done!!"

@REM SET /P ANSWER="sshを実行します。よろしいですか (Y/N)？"
@REM if /i {%ANSWER%}=={y} (goto :yes)
@REM if /i {%ANSWER%}=={yes} (goto :yes)
@REM EXIT
@REM :yes
ssh %USER%@%SERVER%
