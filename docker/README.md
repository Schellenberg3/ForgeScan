# ForgeScan Dockerfiles

Docker is a containerization tool which provides isolated and minimal virtualized environments. It
is useful for testing code and documenting the required environment setup.

## Docker and Nvidia Setup

### Docker dependencies

If you do not have docker installed simply follow [the Docker installation guide](https://docs.docker.com/get-docker/)
for your system.

Note that for Linux systems there is are a few additional [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
to complete.

To verify that Docker is working, run:

```bash
# Linux users should be able to run this without sudo.
docker run --rm hello-world
```

### Nvidia Docker

Getting started with graphics acceleration in Docker can be tricky. However, Nvidia provides a
simple way to interface with CUDA enabled graphics cards via the Nvidia container toolkit. This
must be installed to use CUDA in a container.

You don't need to install Nvidia Docker to build CUDA container. You will need to install Nvidia
Docker to run the CUDA container. Follow the steps in the [Nvidia Container Toolkit instillation guid](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit).
First [install](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installation)
the toolkit and then [configure](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)
it for Docker.

To verify that the Nvidia Docker is working, run:

```bash
# The tag for nvidia/cuda may vary. This project builds from the 12.2.0-devel-ubuntu22.04 image
docker run --rm --gpus all nvidia/cuda:12.2.0-devel-ubuntu22.04 nvidia-smi
```

This is required for using [`Dockerfile.cuda`](Dockerfile.cuda).

## Building these images.

The following commands will tell docker to build the images. **Note that this must be run from the**
**project's root directory**. If the images are build in a different folder then the Dockerfile will
fail to locate the files in the project that they require.

For the CPU-only [`Dockerfile`](Dockerfile).
```bash
docker build --file ./docker/Dockerfile --tag forgescan/ubuntu22.04:cpu .
```

For the CUDA-enabled [`Dockerfile.cuda`](Dockerfile.cuda).
```bash
docker build --file ./docker/Dockerfile.cuda --tag forgescan/ubuntu22.04:cuda .
```

## Trying and using this Project

### In Docker

To test the code in a completely isolated environment, simply use run the desired Docker container:

```bash
# The CPU container:
docker run -it forgescan/ubuntu22.04:cpu bin/bash

# Or the CUDA container:
docker run -it forgescan/ubuntu22.04:cuda bin/bash
```

And inside the container checkout then build and install ForgeScan:
```bash
# the flag --recurse-submodules ensures that HighFive is included
git clone --recurse-submodules https://github.com/Schellenberg3/ForgeScan.git
cd ForgeScan && mkdir build && cd build
cmake ..
make
make install
```

After running this, the directory `ForgeScan/bin` should appear with the compiled example executables.

To exit a container press `ctrl+P+Q`.


### In a Devcontainer

If you plan to edit code or want the files you make to be persistent on the host machine, then a
[Development Container](https://containers.dev/) may be a better choice than a standard Docker
container. In short, this VSCode extension utilizes docker to create an isolated environment but
keeps the source code on your local system.

Learn more about it [here](https://code.visualstudio.com/docs/devcontainers/containers).

To get started, use  VSCode and add a folder in the base of the directory called `.devcontainer`
and create a file called `devcontainer.json` with the following contents.

```json
{
    "name": "ForgeScan Container",

    "build": {
        "context": "..",
        "dockerfile": "../docker/Dockerfile"
    },

    // Required for the CUDA image. Not needed for the CPU-only image.
    "runArgs": [
        "--gpus", "all"
    ],

    // Select the container's non-root user to avoid filesystem issues with root-created objects.
    "remoteUser": "forgescan-dev",

    // *********************** OPTIONAL BUT HELPFUL ADDITIONAL SETTINGS ************************ //

    "customizations": {
        "vscode": {
            "extensions": [
                // Add any VS Code extensions to run in the container: 
                "ms-vscode.cpptools",
                "ms-vscode.cpptools-extension-pack",
                "ms-vscode.cmake-tools",
            ],

            // Ensure the container shell is bash and not sh
            "settings": {
                "terminal.integrated.defaultProfile.linux": "bash", 
                "terminal.integrated.profiles.linux": {
                    "bash": {
                        "path": "/bin/bash"
                    }
                }
            }
        }
    },

    // Prevents erroneous git "detected dubious ownership in repository" warning.
    "postAttachCommand": "git config --global --add safe.directory ${containerWorkspaceFolder}",
}
```

If the images were [built with Docker](#In-Docker) already, then the tag `"build"` may be swapped
for either of the following:

```json
    "image": "forgescan/ubuntu22.04:cpu"
```

```json
    "image": "forgescan/ubuntu22.04:cuda"
```