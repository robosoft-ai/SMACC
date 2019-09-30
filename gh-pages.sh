#!/bin/bash

#---- TEST GHPAGES LOCALLY VARIABLES------
#TRAVIS_BRANCH=master
#TRAVIS_REPO_SLUG=smacc
#GITHUB_TOKEN=
#CATKIN_WORKSPACE_ROOT=`pwd`/../..
# -----------------------------------

# industrial_ci catkin workspace
CATKIN_WORKSPACE_ROOT=/root/target_ws 

DIRECTORY=$(cd `dirname $0` && pwd)
echo $DIRECTORY

echo "GH-PAGES"
if [ -n "$GITHUB_TOKEN" ]; then
    cd "$TRAVIS_BUILD_DIR"

    #find / | grep SMACC
    
    apt-get install -y git

    source $CATKIN_WORKSPACE_ROOT/install/setup.bash
    #source /root/catkin_ws/install/setup.bash

    #remove cloned folder just in case it already existed
    rm -R /tmp/doc

    # This generates a `web` directory containing the website.
    echo "cloning gh-pages"
    git clone https://pabloinigoblasco:$GITHUB_TOKEN@github.com/reelrbtx/smacc --branch gh-pages /tmp/doc

    echo "removing specific branch folder from repo clone.."
    cd /tmp/doc
    rm -r $TRAVIS_BRANCH >/dev/null
    mkdir -p $TRAVIS_BRANCH

    echo "cd $CATKIN_WORKSPACE_ROOT"
    ls $CATKIN_WORKSPACE_ROOT/src
    cd $CATKIN_WORKSPACE_ROOT/src/SMACC
    ls
    
    echo "executing doxygen command"
    doxygen Doxyfile

    echo "moving result files to branch directory..."
    mv /tmp/html /tmp/doc/$TRAVIS_BRANCH
    mv /tmp/latex /tmp/doc/$TRAVIS_BRANCH

    #git init
    #git checkout -b gh-pages
    "cd /tmp/doc/$TRAVIS_BRANCH"
    cd /tmp/doc/$TRAVIS_BRANCH

    git add .
    #git add -f /tmp/doc/index.html
    #echo "------LIST OF FILES ------"
    #find . 
    echo "commiting new documentation"
    
    echo `which git`
    echo $(git -c user.name='travis' -c user.email='travis' commit -q -m "gh-pages travis")
    git -c user.name='travis' -c user.email='travis' commit -q -m "gh-pages travis"

    echo "pushing new documentation"
    
    # Make sure to make the output quiet, or else the API token will leak!
    # This works because the API key can replace your password.
    git push -f -q https://pabloinigoblasco:$GITHUB_TOKEN@github.com/reelrbtx/smacc gh-pages&>/dev/null
    cd "$TRAVIS_BUILD_DIR"
fi
