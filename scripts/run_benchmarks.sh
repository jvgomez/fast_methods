#!/bin/bash

# INSTRUCTIONS
#
# 1. Put all the cfg files you want to execute in fastmarching/benchmark/cfg
# 2. Run $ ./run_benchmarks (or specify the relative route to the script).
# 3. The results folder with logs (and grids) will be stored in the current terminal folder.

BASEDIR=$(dirname $0)
echo $BASEDIR


bold=`tput bold`
normal=`tput sgr0`
FILES=($BASEDIR/../benchmark/cfg/*)

n=0
for f in "${FILES[@]}"
do
    let n++
    echo""
    echo "${bold}Processing file $n out of ${#FILES[@]}${normal}"
    $BASEDIR/../benchmark/build/fmm_benchmark $f
    echo ""
done