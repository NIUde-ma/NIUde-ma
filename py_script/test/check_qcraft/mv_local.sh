#!/bin/bash

REMOTE_HOST="root@10.43.0.28"
REMOTE_PATH="/backhaul/local"
LINK_PATH="/usr/local"

if ssh "$REMOTE_HOST" "[ -d '$REMOTE_PATH' ]"; then
    echo -e "/backhaul/local already exists"
    exit 1
fi

echo "Moving /usr/local to /backhaul and creating symlink..."

ssh "$REMOTE_HOST" "mv '$LINK_PATH' /backhaul/"

ssh "$REMOTE_HOST" "sleep 5 && sync"

ssh "$REMOTE_HOST" "ln -s '$REMOTE_PATH' '$LINK_PATH'"

ssh "$REMOTE_HOST" "sync"

echo "Done! Verifying..."

ssh "$REMOTE_HOST" "ls -la '$LINK_PATH' && echo 'Symlink created successfully'"