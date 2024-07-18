#!/bin/sh

for file in ./*
    do 
    if ! test -f $file
    then
    	echo $file
    	cd $file
    	./make.sh
    fi
done
