# Mpc Docker Image for running tail-sitter MPC

# Bugs
1. Could source nmpc.bash to a .bashrc file? Otherwise another bash will not find config.config etc.
2. Scipy version is not compatible with numpy
```shell
/usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 2.0.0
```
GPT solution is 
```
pip uninstall numpy
pip install numpy==1.24.3
```


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

