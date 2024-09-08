IMAGE_NAME ?= localhost/autoware
AUTOWARE_VERSION ?= $(shell date +'%Y%m%d')
CONTEXT ?= .

info:
	echo "Autoware version: ${AUTOWARE_VERSION}"

base:
	buildah build \
		-f containers/autoware-devel/Dockerfile \
		--build-arg IMAGE_NAME=${IMAGE_NAME} \
		--build-arg AUTOWARE_VERSION=${AUTOWARE_VERSION} \
		--layers=true \
		--target autoware-base \
		--tag autoware:latest-base \
		--tag autoware:${AUTOWARE_VERSION}-base \
	${CONTEXT}

builder: base
	buildah build \
		-f containers/autoware-devel/Dockerfile \
		--build-arg IMAGE_NAME=${IMAGE_NAME} \
		--build-arg AUTOWARE_VERSION=${AUTOWARE_VERSION} \
		--layers=true \
		--target autoware-builder \
		--tag autoware:latest-builder \
		--tag autoware:${AUTOWARE_VERSION}-builder \
	${CONTEXT}

builder-with-cache: builder
	buildah build \
		-f containers/autoware-devel/Dockerfile \
		--build-arg IMAGE_NAME=${IMAGE_NAME} \
		--build-arg AUTOWARE_VERSION=${AUTOWARE_VERSION} \
		--layers=true \
		--target autoware-builder-with-cache \
		--tag autoware:latest-builder-with-cache \
		--tag autoware:${AUTOWARE_VERSION}-builder-with-cache \
	${CONTEXT}

devel: builder-with-cache
	buildah build \
		-f containers/autoware-devel/Dockerfile \
		--build-arg IMAGE_NAME=${IMAGE_NAME} \
		--build-arg AUTOWARE_VERSION=${AUTOWARE_VERSION} \
		--format oci \
		--layers=true \
		--target autoware-devel \
		--tag autoware:latest-devel \
		--tag autoware:${AUTOWARE_VERSION}-devel \
	${CONTEXT}

runtime: builder-with-cache
	buildah build \
		-f containers/autoware-runtime/Dockerfile \
		--build-arg IMAGE_NAME=${IMAGE_NAME} \
		--build-arg AUTOWARE_VERSION=${AUTOWARE_VERSION} \
		--format oci \
		--layers=true \
		--target autoware-runtime \
		--tag autoware:latest-runtime \
		--tag autoware:${AUTOWARE_VERSION}-runtime \
	${CONTEXT}