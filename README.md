# IoT4SPACE
Open Education Plateform to Monitor and Analyze IoT from Space 

# Version
This is a very early version of this platform. It has be badly written and many features are missing.
Be indulgent, report any issue and push any improvement.

## Contributors :
* Fabien Ferrero

Last update: 5/12/2020

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
* Every day, Node-red will access to celestrak.com, and download latest TLE for LS1 and LS2D
* The TXT file will be processed by a Python code once packet from satellite are received. Node-red will run the python code as a system command. The code will write a CSV file. Check that the path is ok. In the python command, the first argument is the file to read, the second argument in the CSV file to write.
* In the python script (lsx_write_auto.py), you must update your home latitude and longitude
* Each time a packet from sat is received, the CSV file with packets received and associated satellite position is updated. You may need to uptdate the CSV file name.
* On the map, live position of LS1 and LS2D is updated each 30s.

![Map](https://github.com/FabienFerrero/IoT4SPACE/blob/master/doc/node-red.jpg)

# Results

In your browser : http://localhost:1800/Worldmap

![Map](https://github.com/FabienFerrero/IoT4SPACE/blob/master/doc/LS1_map_Antibes_test5.jpg)

# Main features
* Store TLE for satellite every day
* Store received packet from a node for terrestrial and satellite communication with associate time and RSSI
* Calculate for each received packet the satellite position, elevation and distance from the node using the corresponding TLE (on daily basis)
* Plot on node-red Worldmap the position of satellite for the different packet
* Plot on node-red Worldmap the live position of satellite

# Actual limitations

* The code works with only two satellites (LS1 and LS2D)
* Each pass on a day is registered in a map as a new layer. By default, layers are hided and have to be activated manually. You can activate all layer by clicking on the star button
* There is no possiblity to filter the results in term of node, date or pass
* RSSI versus elevation and range is not available as a graph


