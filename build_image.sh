LIBRS_VERSION=2.56.5

docker build \
    -t ros/humble-kortex \
    --build-arg LIBRS_VERSION=$LIBRS_VERSION \
    --build-arg USER_UID=1000 \
    --build-arg USER_GID=1000 \
    --build-arg USERNAME=zone \
    --no-cache \
    .