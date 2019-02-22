#!/bin/bash
echo "GH-PAGES"
if [ -n "$GITHUB_TOKEN" ]; then
    cd "$TRAVIS_BUILD_DIR"

    source /root/catkin_ws/devel/setup.bash
    #source /root/catkin_ws/install/setup.bash

    # This generates a `web` directory containing the website.
    echo "cloining gh-pages"
    git clone https://pabloinigoblasco:$GITHUB_TOKEN@github.com/reelrbtx/smacc --branch gh-pages /tmp/doc

    echo "removing specific branch folder from repo clone.."
    cd /tmp/doc
    git rm -r $TRAVIS_BRANCH/$TRAVI_REPO_SLUG >/dev/null
    mkdir -p $TRAVIS_BRANCH/$TRAVI_REPO_SLUG

    #git init
    #git checkout -b gh-pages
    cd $TRAVIS_BRANCH/$TRAVI_REPO_SLUG

    echo "generating roslite documentation"
    rosdoc_lite `rospack find smacc` -o smacc
    rosdoc_lite `rospack find smacc_tool_plugin_template` -o smacc_tool_plugin_template
    rosdoc_lite `rospack find backward_global_planner` -o backward_global_planner
    rosdoc_lite `rospack find backward_local_planner` -o backward_local_planner
    rosdoc_lite `rospack find forward_global_planner` -o forward_global_planner
    rosdoc_lite `rospack find forward_local_planner` -o forward_local_planner
    rosdoc_lite `rospack find smacc_navigation_plugin` -o smacc_navigation_plugin
    rosdoc_lite `rospack find smacc_odom_tracker` -o smacc_odom_tracker
    rosdoc_lite `rospack find smacc_planner_switcher` -o smacc_planner_switcher
    rosdoc_lite `rospack find radial_motion_example` -o radial_motion_example
    git add .
    echo "------LIST OF FILES ------"
    find . 
    echo "commiting new documentation"
    
    git -c user.name='travis' -c user.email='travis' commit -q -m "gh-pages travis"

    echo "pushing new documentation"
    
    # Make sure to make the output quiet, or else the API token will leak!
    # This works because the API key can replace your password.
    git push -f -q https://pabloinigoblasco:$GITHUB_TOKEN@github.com/reelrbtx/smacc gh-pages&>/dev/null
    cd "$TRAVIS_BUILD_DIR"
fi
