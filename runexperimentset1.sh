#!/bin/bash
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
do
    ./runntimes.sh 25 experiments/DDSAPL${i}Searchers.xml results/DDSAPL${i}Searchers.txt
done
