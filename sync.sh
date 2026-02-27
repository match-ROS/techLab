#!/bin/sh
echo "Sending to techLab1"
rsync --delete -avzhe ssh ~/techlab_ws/src rosmatch@techLab1:~/catkin_ws/
rsync --delete -avzhe ssh ~/techlab_ws/src rosmatch@techLab2:~/catkin_ws/

while inotifywait -r -e modify,create,delete,move  ~/techlab_ws/src; do
	echo "Sending to techLab1"
	rsync --delete -avzhe ssh ~/techlab_ws/src rosmatch@techLab1:~/catkin_ws/
done
