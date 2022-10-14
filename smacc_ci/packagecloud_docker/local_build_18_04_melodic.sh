export ROS_VERSION_NAME="melodic"
export UBUNTU_VERSION="bionic"
GITHUB_USER="$1"
GITHUB_TOKEN="$2"
PACKAGE_CLOUD_USER="$3"
PACKAGE_CLOUD_TOKEN="$4"

source /opt/ros/melodic/setup.bash --extend
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export SMACC_SRC_FOLDER=`realpath $DIR/../..`
echo $SMACC_SRC_FOLDER
source $SMACC_SRC_FOLDER/../../devel/setup.bash
python  $SMACC_SRC_FOLDER/smacc_ci/packagecloud_docker/generate_debs.py -ros_version="$ROS_VERSION_NAME" -src_folder="$SMACC_SRC_FOLDER" -smacc_viewer_src_folder="src/SMACC_Viewer" -token="$PACKAGE_CLOUD_TOKEN" -repo_owner="$PACKAGE_CLOUD_USER" -ubuntu_version="$UBUNTU_VERSION"
