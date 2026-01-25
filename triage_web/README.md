<<<<<<< HEAD
# DARPA_triage2026
=======
Requirements

Ubuntu 22.04

ROS 2 Humble

Python 3.10

A working camera (or webcam)

--------------------------------------------------

Terminal A (HOST): start the ROS camera publisher

This runs on your host (not in Docker) and publishes /image.

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.zsh

# publish your webcam to /image
ros2 run image_tools cam2image --ros-args -r image:=/image


Keep this terminal running.

2) Terminal B (HOST): start the Docker UI

From your repo root (where docker-compose.yml is):

docker compose up --build


First time (or after dependency changes), --build is good.

After that, you can run without rebuilding:

docker compose up


Open the UI:

http://127.0.0.1:8080

3) Stop everything cleanly

Stop Docker: Ctrl+C in the compose terminal, then:

docker compose down


Stop cam2image: Ctrl+C in Terminal A.

4) Quick health checks if something looks wrong

Check ROS is publishing (host):

source /opt/ros/humble/setup.zsh
ros2 topic info -v /image


You want Publisher count: 1.

Check Docker can receive (container):

docker exec -it triage_web-triage_ui-1 bash -lc "source /opt/ros/humble/setup.bash && ros2 topic hz /image"


Check Flask is returning bytes:

curl -I http://127.0.0.1:8080/image/drone5


You want HTTP/1.1 200 OK (not 204).

****WORKING ENDPOINTS 
Available Endpoints
Endpoint	Description
/	            Web dashboard
/image	        Latest camera frame (JPEG)
/location	    ROS-based triage location
/ip_location	IP-based operator location
>>>>>>> 19f6686 (Initial DARPA Triage 2026 web GUI with ROS + Flask)
