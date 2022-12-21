#!/bin/bash

if [ ! -d "res" ]
then
    print "This script should be run in the cartesian_impedance_controller package folder. Exiting."
    exit -1
fi

docker run --rm \
    --volume $PWD/res:/data \
    --user $(id -u):$(id -g) \
    --env JOURNAL=joss \
    openjournals/inara \
    -o preprint,pdf,crossref \
    paper.md