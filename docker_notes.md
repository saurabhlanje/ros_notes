# Why docker is faster than virtual box #
Docker do not need separate OS kernal where as virtualbox needs OS kernal. Docker uses same OS kernal from native OS. Docker runs separate OS application layer. So resouorce requirement is very less. Whereas virtualmachine runs OS kernal as well. Hence it needs more bootup time.
# Docker is virtualisation software #
# Images can be distributed easliy #
# NO need to install envronment on own system use docker and install everything in it  #
# Docker can be used from any OS and can be used to virtualize any OS #
# Different versions of same software/environment is possible #
# Docker desktop is for windows and MAC #
# Docker consist of 2 parts #
## Docker Engine or Server ##
## Docker cli or client ##
Gui is also available for docker client
# Image Vs Container #
Image is like a class and Container is like object of that class. Image tell the configuration of os, Container is instance of os configured by image.
# Where to get docker images? From docker registries #
# Docker hub has lot of images #




1) Pull docker image
``` docker pull <image_name>:TAG ```
2) Show all docker images
```docker images```
2) Remove docker image
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

