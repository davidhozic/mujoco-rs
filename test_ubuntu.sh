# Run tests on multiple ubuntu versions, including all features (i.e., with GLFW).

for version in 22.04 24.04 25.04; do
    echo Testing $version
    docker buildx build -t mujoco-rs-ubuntu-$version --build-arg UBUNTU_VERSION=$version -f Dockerfile.ubuntu .
    docker run -it -v ./mujoco-3.3.5:/mujoco/ -t mujoco-rs-ubuntu-$version cargo test --lib --release
done
