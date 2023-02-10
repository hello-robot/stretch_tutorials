# Nav2 Overview

!!! note
    ROS 2 tutorials are still under active development. 

## Overview
The ROS 2 Navigation Stack or Nav2 is a motion planning and control framework for the mobile base. It is a stack because it encompasses several ROS packages that help with mapping, localization, planning and navigation. Stretch's mobile base has been integrated and works well right out of the box with Nav2.

## Motivation
Stretch has a differential drive mobile base that enables navigation. However, before Stretch can navigate, it must be able to generate a map, localize itself in the environment and plan a path between points. This is challenging when the environment could present challenges such as static or dynamic obstacles that can get in the way of the robot, or floor surfaces with different frictional properties that can trip a controller.

Fortunately, the Nav2 stack enables these capabilities through various packages and provides a simple Python API to interact with them. Let’s take a quick tour!

## Demo with Stretch

### Mapping
It is possible to generate a high-fidelity 2D map of your environment using the slam_toolbox package. For you, it is as easy as teleoperating Stretch in your home, laboratory or office. Stretch does the rest.

<p align="center">
  <img height=500 src="https://user-images.githubusercontent.com/97639181/206606439-a3e346d4-83d9-45ec-93cc-8804a2b9719c.gif"/>
</p>

### Navigation
Once a map has been generated, Stretch can localize itself and find its way in its environment. Just point on the map and Stretch will get there, without your help.

<p align="center">
  <img height=500 src="https://user-images.githubusercontent.com/97639181/206606699-9f3b87b1-a7d1-4074-b68a-2e880fc576a3.gif"/>
</p>

### Simple Commander Python API:
It is as simple to interact with Nav2 programmatically, thanks to the Python API. For example, have a look at this neat patrol demo.

<p align="center">
  <img height=500 src="https://user-images.githubusercontent.com/97639181/214690474-25d2bbf5-74b2-4789-af7e-ef6b208b2275.gif"/>
</p>

Have a look at the following tutorials to explore the above capabilities on your own Stretch! If you stumble upon something unexpected, please let us know.