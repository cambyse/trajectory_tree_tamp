# Building the TAMP examples

For a straightforward installation that works seamlessly on most Ubuntu systems (20.04 and later) and macOS, we recommend using the Dockerized setup.

Alternatively, a native system installation is possible but may require additional steps to have all correct dependencies necessary to compile the examples.

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

This creates a docker image called `tamp`. It installs the dependencies, clones repositories and compiles the examples. 


2. Run Docker image

```bash
docker run --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it tamp /bin/bash
```

The above command runs the docker image and opens a bash terminal. The options `--net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` are to forward the host display to the Docker container.

## Building the examples on the native system

To build the examples on the native system, one has to follow the same steps as in Dockerfile (https://github.com/cambyse/trajectory_tree_tamp_deployment). ⚠️ This is tested only on `Ubuntu 20.04.6 LTS`.

1. Install dependencies

```bash
sudo apt update && sudo apt install -y git build-essential \
 liblapack-dev \
 mesa-common-dev \
 libeigen3-dev \
 freeglut3-dev \
 libf2c2-dev \
 libjsoncpp-dev \
 libqhull-dev \
 libann-dev \
 libassimp-dev \
 libglew-dev \
 libglfw3-dev \
 libgtest-dev \
 libopencv-dev \
 libboost-all-dev \
 cmake
```

2. Clone repository and its submodules
```bash
git clone https://github.com/cambyse/trajectory_tree_tamp.git trajectory_tree_tamp && cd trajectory_tree_tamp
git submodule update --init --recursive
```

3. Build Rai

```bash
cd rai
make
```

4. Build TAMP libraries and examples

```bash
cd ../share/projects
mkdir 17-camille-obsTask_build && cd 17-camille-obsTask_build
cmake ../17-camille-obsTask -DMLR_LIBRARIES_DIR=../../../rai/lib -DMLR_INCLUDE_DIR=../../../rai/rai -DCMAKE_BUILD_TYPE=Release
make
```

# Run examples
The examples are placed in the `apps` folder .

### Baxter-A
```bash
cd lgp-tree-po-blocks-baxter
LIBGL_ALWAYS_SOFTWARE=1 ./lgp-tree-po-blocks-baxter -pb A -c0 1.0
```

The argument `c0` can be adjusted to allow more or less exploration. 
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


