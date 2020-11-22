# IoT4SPACE
Open Education Plateform to Monitor and Analyze IoT from Space 

# Version
This is a very early version of this platform. It has be badly written and many features are missing.
Be indulgent, report any issue and push any improvement.

## Contributors :
* Fabien Ferrero

Last update: 22/11/2020

## Pre-requisites

IOT4SPACE has been tested on Ubuntu 20.04.
It requires :
* [Node-RED](https://nodered.org) to be installed
* node-red-contrib-ttn Modules in Node-red (https://flows.nodered.org/node/node-red-contrib-ttn)
* and node-red-contrib-web-worldmap Modules in Node-red (https://flows.nodered.org/node/node-red-contrib-web-worldmap)
* Python3 to be installed (https://www.python.org/download/releases/3.0/)
* Piephem python library (https://pypi.org/project/pyephem/)

# Install

At first, install any pre-requisites software and module

* Import JSON in your node-red Flow
* Set your Device and Application ID
* Check the path to python program
* Wait for receiving some data

# Configure

* Configure your node and application in TTN uplink node
* The flow will save data from the node in a TXT file. You can change the name
* The TXT file will be processed periodically by a python code. Node-red will run the python code as a system command. The code will write a CSV file.Check that the path is ok. In the python command, the first argument is the file to read, the second argument in the CSV file to write.
* In the python script (ls1_write.py), you have to update TLE and your local coordinate
* The last part of the Node-red code will periodically read and plot the CSV file. Uptade the CSV file name.

![Map](https://github.com/FabienFerrero/IoT4SPACE/blob/master/doc/node-red.jpg)

# Results

In your browser : http://localhost:1800/Worldmap

![Map](https://github.com/FabienFerrero/IoT4SPACE/blob/master/doc/LS1_map_Antibes_test5.jpg)

# Actual limitations

* The code works only for one satellite
* Need up to 1mn to populate the map
* There is no possiblity to filter the results in term of node, date or pass

