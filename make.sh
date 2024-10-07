#!/bin/sh
cd base
./make.sh
cd ..

cd app
./make.sh
cd ..

rm -rf ../../sim_module_pkg/configs/*
cp -r configs/* ../../sim_module_pkg/configs/

rm -rf ../../sim_module_pkg/robots/*
cp -r robots/* ../../sim_module_pkg/robots/