#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm -f .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln session.yml .tmuxinator.yml

# per-aircraft setup file (exports FCU_URL etc., sources shell_helpers.sh)
SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=rw_setup.sh

# start tmuxinator
tmuxinator ibvs_perching_rw setup_name=$SETUP_NAME
