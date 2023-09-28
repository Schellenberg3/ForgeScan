# Share

This folder contains items used by ForgeScan executables. By default, both the [Example](../src/Examples/)
and [Experiment](../src/Experiments/) executables will search for files in this directory and save
their outputs within it too. The Python [scripts](../scripts/) also assume specific files from the
executables have been saved here.

The required folders will be created automatically. And outputs are ignored by git.

The [Meshes](./Meshes/) directory specifically includes some example meshes. Some meshes were created
specifically for this project. They demonstrate operation on simple or specific objects. To demonstrate
operation on a wider variety of manufacturing themed objects, some additional models from the ABC 
[ABC dateset](https://deep-geometry.github.io/abc-dataset/) of CAD models are also included.

For more details see [Meshes](./Meshes/README.md).
