# Autoware Container Images

This repository hosts a collection of experimental Autoware container images. These images are designed to provide a convenient and reproducible environment for developing and running Autoware, an open-source software stack for autonomous driving.

### Runtime images

The image `ghcr.io/bounverif/autoware:latest` provides a minimal runtime environment and binaries. 

Thes images are deployment candidates, which will be tested systematically before any deployment. 

> [!CAUTION]
> This image is currently under development. Currently not ready for execution.

### Builder images

The image `ghcr.io/bounverif/autoware:latest-builder` provides a build environment for Autoware where necessary build tools and dependencies are pre-installed. 

The image `ghcr.io/bounverif/autoware:latest-builder-with-cache` additionally includes a buildcache to speed up your Autoware builds.

These images are built weekly and recommended for the CI use.

### Devel images

The image `ghcr.io/bounverif/autoware:latest-devel` provides a richer development environment for Autoware developers. 

We suggest developers using the `distrobox` tool for the best developer experience when developing with these images.

> [!WARNING]
> This image is currently under development. Provides minimal extra functionality over builder images.

## License

This repository is licensed under the [Apache License 2.0](LICENSE).
