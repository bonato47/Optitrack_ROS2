#!/bin/bash
docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   -u root \
	   --net host \
	   epfl-lasa/optitrack_zmq
