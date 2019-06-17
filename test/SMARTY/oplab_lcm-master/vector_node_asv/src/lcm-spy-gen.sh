#!/bin/bash
# Script to enable lcm-spy to decode packets

# ask lcm-java package for classpath, using pkg-config system tool
LCM_JAR=`pkg-config --variable=classpath lcm-java`

if [ $? != 0 ] ; then
    if [ -e /usr/local/share/java/lcm.jar ] ; then
        LCM_JAR=/usr/local/share/java/lcm.jar
    else
        if [ -e ../../lcm-java/lcm.jar ] ; then
            LCM_JAR=../../lcm-java/lcm.jar
        fi
    fi
fi

# Emit Java code for lib_sensors
lcm-gen -j lib_sensors/*.lcm
javac -cp $LCM_JAR lib_sensors/image/*.java
jar cf my_types.jar lib_sensors/image/*.class
javac -cp $LCM_JAR lib_sensors/sensehat/*.java
jar cf my_types.jar lib_sensors/sensehat/*.class
javac -cp $LCM_JAR lib_sensors/strain/*.java
jar cf my_types.jar lib_sensors/strain/*.class

# Emit Java code for lib_control
lcm-gen -j lib_control/*.lcm
javac -cp $LCM_JAR lib_control/control/*.java
jar cf my_types.jar lib_control/control/*.class
javac -cp $LCM_JAR lib_control/thruster/*.java
jar cf my_types.jar lib_control/thruster/*.class

# Emit Java code for lib_navigation
lcm-gen -j lib_navigation/*.lcm
javac -cp $LCM_JAR lib_navigation/navigation/*.java
jar cf my_types.jar lib_navigation/navigation/*.class