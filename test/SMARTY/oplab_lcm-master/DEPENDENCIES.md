# oplab_lcm dependencies map

This section contains a visual representation of the dependencies among the different modules/libraries of *oplab_lcm*. The SVG files are generated using [graphviz](https://graphviz.org/) from *main.dot*, *libraries.dot* and *lcm-map.dot*. To do so, just type:
	
	neato -Tsvg libraries.dot > lib-diagram.svg
	neato -Tsvg main.dot > main-diagram.svg
	neato -Tsvg lcm-map.dot > lcm-map.svg

The modules contains different type of files: bash scripts, Python source code and lcm message definitions. All the relevant files have been classified in the following sections:

- [Main sources](#main-sources)
- [Sensors](#sensors)
- [Control, navigation and safety](#control-navigation-and-safety)
- [The lcm handler](#lcm-handler)

## Main sources
Contains all the bash scripts, YAML configuration files and the **asv_main.py** module

## Sensors
Contains the libraries required to the camera, straing gauge, visual tracking, Qualisys and the SenseHat

## Control, navigation and safety
Contains the low-mid level control of the thrusters and its driving board, the PID controller and also the navigation modules for tracking and Qualisys.

## lcm handler
Contains the specific modules **lcm_handler.py** and **config.py** located in _vector_node_asv/src/lcm_handler_

# Dependencies diagrams

Main source code diagram
![Main sources diagram](/doc/main-diagram.svg)

Complete libraries diagram
![Libraries diagram](/doc/lib-diagram.svg)

Map of LCM channels and nodes for interprocess communication
![lcm map](/doc/lcm-map.svg)
