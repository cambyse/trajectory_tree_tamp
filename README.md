# Installation

For a straightforward installation that works seamlessly on most Ubuntu systems (20.04 and later) and macOS, we recommend using the Dockerized setup.

Alternatively, a native system installation is possible but may require additional steps to build the examples successfully.

## Installation using Docker
1. Clone repository containing the Docker file

```bash
git clone git@github.com:cambyse/trajectory_tree_tamp_deployment.git
```

2. Build Docker image

```bash
sudo docker build -t tamp .
```

2. Run Docker image

```bash
sudo docker run docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <image_name> -it tamp /bin/bash
```

## Installation without Docker


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


