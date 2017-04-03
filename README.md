# DQ-N 
This is a implementation of DQ-N protocol
--

## Install
First you need to clone this repo recursively as it references the RadioHead library
```
$ git clone --recursive git@github.com:Kuree/DQN.git 
```
If you are going to use it on the Arduino based board, you need to add [ETL library](http://www.etlcpp.com/). Once you follow the instruction to add ETL to your Arduino library folder, you need to link these two following libraries to Arduino library folder as well:
```
$ ln -s core $ARDUINO_PATH/libraries/protocol
$ ln -s server/RadioHead $ARDUINO_PATH/libraries/RadioHead
```
You shall replace ```$ARDUINO_PATH``` with your installment path.

## Examples
This repo contains two examples, one for the server and one for the deivce. To see how to use the server, see the ```server``` folder and the ```device``` folder for device examples. The server implementation is Arduino compatible and you can load the symbolic link file to bypass the Arduino directory restriction.
