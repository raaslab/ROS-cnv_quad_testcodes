#!/bin/bash
cd ~/softwares/jMAVSimNew/jMAVSim
java -Djava.ext.dirs= -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -serial /dev/ttyACM0 921600 -qgc

