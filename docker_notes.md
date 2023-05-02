1) Show all docker images
```docker images```
2) Remote docker image
``` docker image rm <image_name>```
3) Add image to docker from repository
```docker pull <image_name>```
4) Run docker image. This creates new container for image
``` sudo docker run -it osrf/ros:humble-desktop ```
5) Get details of running docker imagees
``` docker ps -l ```
6) Restart docker
``` sudo systemctl daemon-reload ```
```sudo systemctl restart docker```
7) Start docker container with ID of container
```sudo docker start be3f52685c49```
8) Remove docker container
```docker rm container-name```
9) List all docker containers
```docker ps -a ```
11) Stop some container
```docker stop registry-v1```
12) Image is just the confoguration that is defined as per the docker file. Container is the instance of the image. One image can have multiple containers
13) Connect tp container from ID of container
```sudo docker exec -it be3f52685c49 /bin/bash```
14) 

