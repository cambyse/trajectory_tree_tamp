# Building the TAMP examples

For a straightforward installation that works seamlessly on most Ubuntu systems (20.04 and later) and macOS, we recommend using the Dockerized setup.

Alternatively, a native system installation is possible but may require additional steps to build the examples successfully.

## Building using Docker
1. Clone repository containing the Docker file (https://github.com/cambyse/trajectory_tree_tamp_deployment)

```bash
git clone git@github.com:cambyse/trajectory_tree_tamp_deployment.git
```


2. Build Docker image

```bash
cd trajectory_tree_tamp_deployment
docker build -t tamp .
```

This creates a docker image called 'tamp'. The repositories are checked out and built in this step.


2. Run Docker image

```bash
docker run --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it tamp /bin/bash
```

The above command runs the docker image and opens a bash terminal. The options `--net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` are to forward the display.

## Building the examples of the native system

To 

# Run examples

### Baxter-A
```bash
```

### Baxter-B
```bash
```

### Baxter-C
```bash
```

### Franka-A
```bash
```

### Franka-B
```bash
```

### Franka-CxA'
```bash
```


