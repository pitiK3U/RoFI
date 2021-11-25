#!/bin/bash

while getopts c:s:b: flag
do
    case "${flag}" in
        c) configDir=${OPTARG};;
        s) step=${OPTARG};;
        b) bound=${OPTARG};;
    esac
done

echo "Given arguments:";
echo "ConfigDir: $configDir";
echo "Step: $step";
echo "Bound: $bound";
echo "";

for config in $configDir/*-init.in; do
    config=${config%-init.in};
    result="./results/${config#"$configDir/"}";
    echo "Current config: $config-*.in";

    echo "V-Reconfig: ";
    time /home/sharpain/RoFI/build.Release/desktop/bin/rofi-reconfig -p $bound -s $step -i "$config-init.in" -g "$config-goal.in" -a bfs 1>"$result-old.in";

    echo "R-ISO-Reconfig: ";
    time /home/sharpain/RoFI/build.Release/desktop/bin/rofi-isoreconfig --bound=$bound --step=$step --start="$config-init.in" --target="$config-goal.in" 1>"$result-iso.in";
    
    echo "";
done
