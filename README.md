# Recycle robot server 2
This repository is the second edition of the server code used in the robot vision system for disassembling laptops.

This repository is designed to be run on a raspberry pi provides a control interface for the client to access the robot and camera without the need for direct physic connections. Ideally much of the client code could be run on the (currently server) device and have behaviour more like an edge computing device.

To start:

Connect robotic arm to pi and power up
Place robotic arm in an area it won't collide with other objects.

Run: python3 server.py

wait for the beep after the robot has move to central location before contacting the server
