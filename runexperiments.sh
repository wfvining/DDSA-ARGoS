#!/bin/bash
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
do
    ./runntimes.sh $1 experiments/DSAPL${i}Searcher.xml final_results/DSAPL${i}Searchers.txt
done

for i in 0 1 2 3 4 5 6 7 8 9 10
do
    ./runntimes.sh $1 experiments/DSAPLError${i}.xml final_results/DSAPLError${i}.txt
done

for i in 0 1 2 3 4 5 6 7 8 9 10
do
    ./runntimes.sh $1 experiments/DSACError${i}.xml final_results/DSACError${i}.txt
done

for i in 0 1 2 3 4 5 6 7 8 9 10
do
    ./runntimes.sh $1 experiments/DSAUError${i}.xml final_results/DSAUError${i}.txt
done

for i in 20 40 60 80 100 120 140 160 180 200 220 240 260
do
    ./runntimes.sh $1 experiments/DSAU60minTargets${i}.xml final_results/DSAU60minTargets${i}.txt
done

for i in 5 6 7 8 9 10 11 12 13 14 15
do
    ./runntimes.sh $1 experiments/DSAU60minSize${i}x${i}.xml final_results/DSAU60minSize${i}x${i}.txt
done

