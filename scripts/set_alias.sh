#!/bin/bash

path=$(dirname $BASH_SOURCE)

# usage: launch_base_system <map name>
alias launch_base_system=". ${path}/base_system.sh"

# usage: launch_mppi
alias launch_mppi=". ${path}/mppi.sh"

# usage: launch_slam <map name>
alias launch_slam=". ${path}/slam.sh"

# usage: launch_recalc_trj <map name>
alias launch_recalc_trj=". ${path}/recalc_trj.sh"