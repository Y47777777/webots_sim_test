#!/bin/sh
cd base
./make.sh
cd ..

cd app
./make.sh
cd ..

rm -rf ../../sim_mode_pkg/configs/*
cp -r configs/* ../../sim_mode_pkg/configs/

rm -rf ../../sim_mode_pkg/robots/*
cp -r robots/* ../../sim_mode_pkg/robots/