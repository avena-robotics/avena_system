#!/bin/bash

sudo apt-get install openssh-server -y
sudo apt-get install mc -y
sudo apt-get install nmap -y
sudo apt-get install git -y

# docker compose
sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# nvidia docker
curl https://get.docker.com | sh
sudo systemctl start docker && sudo systemctl enable docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

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

docker stop -t 5 ${docker ps -aq}
docker rm -f ${docker ps -aq}
docker rmi ${docker images -aq}

