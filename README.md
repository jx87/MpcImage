# Mpc Docker Image for running tail-sitter MPC

# Recommendation
## About Networking
since the git clone operation always fails, the more appropriate solution is  clone `acados` repository (including its submodules) to this repository,
and copy files into the container.

## About Environment Variables
`WarriorHanamy` add a file named `nmpc_setup.bash` to the repository, please modify it as required to meet the needs of the project!.

# Install Docker

```shell
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
## Run docker as a non-root user
```shell
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
# Log in/out again before using docker!
```

# Build

```shell
docker build .
```

# Create a image tagged MPCimage
```shell
docker build -f Dockerfile -t MPCimgae
```

# Create a container named myMPC
```shell
docker run -d -it --name myMPC MPCimgae /bin/bash
```

