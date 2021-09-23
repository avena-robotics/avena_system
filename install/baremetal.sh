#!/bin/bash

#few tools
apt-get install openssh-server -y
apt-get install mc -y
apt-get install nmap -y
apt-get install git -y

# docker compose
curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

# nvidia docker
curl https://get.docker.com | sh
systemctl start docker && sudo systemctl enable docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list
apt-get update
apt-get install -y nvidia-docker2
systemctl restart docker

#add local repository server (avena LAN only)
echo '{
    "insecure-registries":["10.3.14.14:5000"],
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}' > /etc/docker/daemon.json

sudo systemctl restart docker

#clean up all dockers - nevermind when errors
docker stop -t 5 ${docker ps -aq}
docker rm -f ${docker ps -aq}
docker rmi ${docker images -aq}

#install newest nvidia drivers
apt install nvidia-utils-470

