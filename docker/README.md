# Docker

To run parrot_ros on a container first install Docker Engine on your station:

```
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

```
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```


Start Docker service:

```
sudo service docker start
sudo docker run hello-world
```

## User Group

To create images you need to add docker to your user group:

```
sudo usermod -aG docker ${USER}
```


## Build

To build the parrot_container using the existing Docker and compose files simply run:

```
docker compose build
```

Once the image is built, run the following commnad to start the container:

```
docker compose up -d
```

Now, to have an interactive terminal over the container run the following:

```
docker exec -it parrot_container bash
```

##TODO

- Run catkin_make on container
- Fix catkin bugs