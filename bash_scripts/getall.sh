#!/bin/bash

filelist=`ls ../img/cg_test/sample${1}/left*rect_filtered.png|wc -l`
echo $filelist files
for (( i=0; i<$filelist; i++ ))
do
    ./testVolumeSubstraction ../img/cg_test/calib/camera_param_left.yaml ../img/cg_test/sample${1}/left_000_rect_16filtered.xml ../img/cg_test/sample${1}/left_00${i}_rect_16filtered.xml > output.txt
    mv output.txt ../img/cg_test/sample${1}/result_00${i}.txt
done
