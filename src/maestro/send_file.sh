#!/bin/bash

# arg1 : filename
TARGET="./"
NAME="example.pexe"
LOCAL_PATH="$TARGET/$NAME"

# scp to remote server
SERVER="192.168.2.110"
USER="user"
PASS="user"
REMOTE_PATH="/mnt/jffs/usr/$NAME"

echo "sending file to $SERVER  :  ...-> $REMOTE_PATH"
scp "$LOCAL_PATH" "$USER"@"$SERVER":"$REMOTE_PATH"
echo "upload done!!"

# sshして書き込み先のファイルを実行できるようにする
echo "ssh to $SERVER and chmod +x $REMOTE_PATH"
ssh "$USER"@"$SERVER" chmod +x "$REMOTE_PATH"
echo "done!!"

read -p "sshを実行します。よろしいですか (Y/N)？" ANSWER
if [[ "$ANSWER" == "y" ]] || [[ "$ANSWER" == "Y" ]]; then
    ssh "$USER"@"$SERVER" source dotconfig_source.bashrc
fi

