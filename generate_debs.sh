

#!/bin/bash
# smacc_msgs smacc 
# smacc_sm_examples smacc_logic_units smacc_diagnostics smacc_simulation

 function build_package {
    echo "Building debianpackage: $1" 
    TARGET_PKG=$1

    bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic $TARGET_PKG
    cp -R debian $TARGET_PKG
    cd $TARGET_PKG
    fakeroot debian/rules build
    rm -R debian/ obj-x86_64-linux-gnu/
    cd ..
    echo "Finding debian file for package: $TARGET_PKG"
    echo "Current directory: $(pwd)"
    FILTER=`echo "$TARGET_PKG" | sed "s/_/-/g"`
    echo "Filter string: $FILTER"
    DEBIANFILE=`find . | grep $FILTER`
    echo $DEBIANFILE
    #sudo gdebi  $DEBIANFILE | yes
    sudo gdebi --option=APT::Get::force-yes,APT::Get::Assume-Yes $DEBIANFILE

}  




#---------- SMACC CLIENT LIBRARY -----------------------
cd smacc_client_library/
for TARGET_PKG in smacc_action_client_generic smacc_interface_components smacc_sensors 
do
build_package $TARGET_PKG
done
mv *.deb ..
cd ..

#---------- SMACC NAVIGATION -----------------------
cd smacc_client_library/smacc_navigation
echo `pwd`

for TARGET_PKG in forward_global_planner backward_global_planner smacc_navigation_plugin smacc_planner_switcher backward_local_planner forward_local_planner smacc_odom_tracker
do
build_package $TARGET_PKG
done
mv *.deb ../..
cd ../..

#---------- smacc_sm_examples -----------------------
cd smacc_logic_units/
for TARGET_PKG in event_aggregator
do
build_package $TARGET_PKG
done
mv *.deb ..
cd ..


#---------- smacc_sm_examples -----------------------
cd smacc_sm_examples/
for TARGET_PKG in sm_dance_bot sm_hello_world_example sm_history_example sm_radial_motion sm_smacc_tutorial_examples
do
build_package $TARGET_PKG
done
mv *.deb ..
cd ..

#---------- ROOT -----------------------
for TARGET_PKG in smacc_msgs smacc 
do
build_package $TARGET_PKG
done





