#/bin/bash -eu

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0})/../../; pwd)
echo "Run sync script in $SRC_DIR"
rsync -av --delete $SRC_DIR/MobileRobotUITutorialProject/Assets/Scripts/ $SRC_DIR/UnityScripts/Scripts
rsync -av --delete $SRC_DIR/MobileRobotUITutorialProject/Assets/OdometryViewer/ $SRC_DIR/UnityScripts/OdometryViewer
rsync -av --delete $SRC_DIR/MobileRobotUITutorialProject/Assets/PointCloud/ $SRC_DIR/UnityScripts/PointCloud
