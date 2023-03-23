#!/bin/bash
# copy_arm_pkg.sh

# Stop on errors
# See https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

usage() {
    echo "Usage: $0 package direction"
    echo -e "    package: mrover_arm | sa_arm"
    echo -e "    direction:"
    echo -e "\tin: Copy from catkin workspace to mrover package"
    echo -e "\tout: Copy from mrover package to catkin workspace"
}

if [ $# -ne 2 ]; then
    usage
    exit 1
fi

rospack find mrover > /dev/null
declare mrover_pkg=`rospack find mrover`

# Parse argument.  $1 is the first argument
case $2 in
    "in")
        case $1 in
            "mrover_arm")
                # check that source directories exist
                if [ ! -d $mrover_pkg/../mrover_arm ]; then
                    echo "Error: mrover_arm package doesn't exist in catkin_ws/src/"
                    exit 1
                fi

                if [ ! -d $mrover_pkg/../mrover_arm/rover_description/ ]; then
                    echo "Error: mrover_arm package is missing rover_description/"
                    exit 1
                fi

                # remove existing directories
                if [ -d $mrover_pkg/src/teleop/mrover_arm/ ]; then
                    rm -r $mrover_pkg/src/teleop/mrover_arm/
                fi

                if [ -d $mrover_pkg/rover_description/arm/ ]; then
                    rm -r $mrover_pkg/rover_description/arm/
                fi

                # copy directories
                cp -r $mrover_pkg/../mrover_arm/. $mrover_pkg/src/teleop/mrover_arm/
                cp -r $mrover_pkg/src/teleop/mrover_arm/rover_description/. $mrover_pkg/rover_description/arm/
                rm -r $mrover_pkg/src/teleop/mrover_arm/rover_description/
                ;;

            "sa_arm")
                # check that source directories exist
                if [ ! -d $mrover_pkg/../mrover_sa_arm/ ]; then
                    echo "Error: mrover_sa_arm package doesn't exist in catkin_ws/src/"
                    exit 1
                fi

                if [ ! -d $mrover_pkg/../mrover_sa_arm/rover_description/ ]; then
                    echo "Error: mrover_sa_arm package is missing rover_description/"
                    exit 1
                fi

                # remove existing directories
                if [ -d $mrover_pkg/src/teleop/mrover_sa_arm/ ]; then
                    rm -r $mrover_pkg/src/teleop/mrover_sa_arm/
                fi

                if [ -d $mrover_pkg/rover_description/sa_arm/ ]; then
                    rm -r $mrover_pkg/rover_description/sa_arm/
                fi

                # copy directories
                cp -r $mrover_pkg/../mrover_sa_arm/. $mrover_pkg/src/teleop/mrover_sa_arm/
                cp -r $mrover_pkg/src/teleop/mrover_sa_arm/rover_description/. $mrover_pkg/rover_description/sa_arm/
                rm -r $mrover_pkg/src/teleop/mrover_sa_arm/rover_description/
                ;;

            *)
                usage
                exit 1
                ;;

        esac
        ;;

    "out")
        case $1 in
            "mrover_arm")
                # check that source directories exist
                if [ ! -d $mrover_pkg/src/teleop/mrover_arm/ ]; then
                    echo "Error: mrover_arm/ directory doesn't exist in src/teleop/"
                    exit 1
                fi

                if [ ! -d $mrover_pkg/rover_description/arm/ ]; then
                    echo "Error: arm/ directory doesn't exist in rover_description/"
                    exit 1
                fi

                # remove existing package
                if [ -d $mrover_pkg/../mrover_arm/ ]; then
                    rm -r $mrover_pkg/../mrover_arm/
                fi

                # copy directories
                cp -r $mrover_pkg/src/teleop/mrover_arm/. $mrover_pkg/../mrover_arm/
                cp -r $mrover_pkg/rover_description/arm/. $mrover_pkg/../mrover_arm/rover_description/
                ;;

            "sa_arm")
                # check that source directories exist
                if [ ! -d $mrover_pkg/src/teleop/mrover_sa_arm/ ]; then
                    echo "Error: mrover_sa_arm/ directory doesn't exist in src/teleop/"
                    exit 1
                fi

                if [ ! -d $mrover_pkg/rover_description/sa_arm/ ]; then
                    echo "Error: sa_arm/ directory doesn't exist in rover_description/"
                    exit 1
                fi

                # remove existing package
                if [ -d $mrover_pkg/../mrover_sa_arm/ ]; then
                    rm -r $mrover_pkg/../mrover_sa_arm/
                fi

                # copy directories
                cp -r $mrover_pkg/src/teleop/mrover_sa_arm/. $mrover_pkg/../mrover_sa_arm/
                cp -r $mrover_pkg/rover_description/sa_arm/. $mrover_pkg/../mrover_sa_arm/rover_description/
                ;;
            
            *)
                usage
                exit 1
                ;;

        esac
        ;;
    
    *)
        usage
        exit 1
        ;;

esac