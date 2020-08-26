#!/bin/bash
cd Documents
mkdir RawData
for i in $(ls |grep Matrix)
do
    # echo $i".txt"
    if [[ $i ==  *"depth"* ]]
        then
        # echo "depth ==== "$i".txt"
        cat $i |grep real|cut -f 2 -d ">"|cut -f 1 -d "<" > $i".txt"

    elif [[ $i =~ "confidence" ]]
        then
        # echo "confidence ==== "$i".txt"
        cat $i |grep integer|cut -f 2 -d ">"|cut -f 1 -d "<" > $i".txt"
    
    fi
    mv $i ./RawData
done
