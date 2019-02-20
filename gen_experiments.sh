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

iterations=5
let seed=$RANDOM

for size in 10 20 40 60 80 100
do
    for clustersize in 4 8 16
    do
        for r in 0.8 0.2
        do
            for ((i=0; i < $iterations; i++))
            do
                file=DDSA_n1_c${clustersize}_R${size}_r${r}_${tag}_i${i}.argos
                erb -T - \
                    csize=${clustersize} \
                    size=${size} \
                    detectionradius=${r} \
                    seed=${seed} \
                    n=1 \
                    -- experiments/DDSA_template.argos.erb \
                    >${experimentdir}/${file}
                echo "cd ~/research/DDSA-ARGoS; argos3 -c ${experimentdir}/$file > results_${tag}/${file}.results; cd ../.."
                let seed++
            done
        done
    done
done
