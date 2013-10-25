"Objectify" plugin for CINEMA 4D
=============
Very much work in progress!

This an attempt to integrate <a href="http://www.pointclouds.org/">pcl</a> pointcloud library into Cinema 4D. Currently the intent is to take a bunch of points and run a fast surface reconstruction from pcl to produce a polygonal object

#[Download](https://github.com/eighteight/Objectify/archive/master.zip)
![Screenshot](https://raw.github.com/eighteight/Objectify/master/screenshot.png)
###[Video](https://vimeo.com) to follow

##Compability
The plugin has been compiled with the R14 sdk and has only been tested with C4D R14.

##Installation
Unzip the folder to your CINEMA/plugins/ folder and restart CINEMA.
You will have to have a pcl 1.7 installed for this plugin to work currently (I built pcl 1.7 using homebrew)


##Usage
Drop a multisegmented spline into the Object field -- it should reconstruct the surface from the cross section of these segements


##Settings
###Objectify


