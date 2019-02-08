#!/bin/bash

if [ $# != 1 ]
then
    echo "please provide an experiment tag"
    exit 1
fi

tag=$1
experimentdir=experiments_${tag}
if ! [ -d ${experimentdir} ]
then
    mkdir ${experimentdir}
fi

if [ -d results_${tag} ]
then
    rmdir results_${tag}
fi
mkdir results_${tag}

iterations=10
let seed=$RANDOM

for size in 10 20 40 80 160 320
do
    for clustersize in 4 8 16
    do
        for r in 1.6 1.2 0.8 0.6 0.4 0.3 0.2 0.15 0.1
        do
            for ((i=0; i < $iterations; i++))
            do
                file=DDSA_n1_c${clustersize}_R${size}_r${r}_${tag}.argos
                erb -T - \
                    csize=${clustersize} \
                    size=${size} \
                    detectionradius=${r} \
                    seed=${seed} \
                    n=1 \
                    -- experiments/DDSA_template.argos.erb \
                    >${experimentdir}/${file}
                echo "argos3 -c ${experimentdir}/$file > results_${tag}/${file}.results"
            done
        done
    done
done
