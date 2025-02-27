#!/bin/sh

sudo rm /srv/tftp/*
sudo cp $HOME/MA35D1_Buildroot-master/output/images/uImage /srv/tftp/
sudo cp $HOME/MA35D1_Buildroot-master/output/images/nuc980-custom.dtb /srv/tftp/nuc980.dtb
