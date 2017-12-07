#!/bin/bash

# INSTRUCTIONS
#
# 1. Put all the cfg files you want to execute in the same folder
# 2. Run $ ./<path_to_scripts>/run_benchmarks <path_to_cfg_files>
# 3. The results folder with logs (and grids) will be stored in the current terminal folder.
#
# NOTE: This script only works for BASH>=4.0 For previous versions: http://stackoverflow.com/a/29237131/2283531

SCRDIR=$(dirname $0)

bold=`tput bold`
normal=`tput sgr0`

shopt -s globstar
files=("$1"/**/*.cfg)

n=0
for f in ${files[@]}
do
    let n++
    echo""
    echo "${bold}Processing file $n out of ${#files[@]}${normal}"
    echo $f
    $SCRDIR/../build/fm_benchmark $f
    echo ""
done
