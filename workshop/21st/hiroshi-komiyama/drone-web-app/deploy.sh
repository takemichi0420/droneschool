#!/bin/bash

# drone-web-app Docker Deploy Script
# Usage: ./deploy.sh <docker-username> [version]

set -e

DOCKER_USERNAME="${1:-yourusername}"
VERSION="${2:-1.0.0}"
IMAGE_NAME="drone-web-app"
REGISTRY="${DOCKER_USERNAME}"

if [ "$DOCKER_USERNAME" = "yourusername" ]; then
    echo "❌ Error: Please provide your Docker Hub username"
    echo "Usage: ./deploy.sh <docker-username> [version]"
    echo "Example: ./deploy.sh hiroshi 1.0.0"
    exit 1
fi

echo "🔨 Building and deploying $IMAGE_NAME to Docker Hub..."
echo "Registry: ${REGISTRY}/${IMAGE_NAME}"
echo "Version: ${VERSION}"
echo ""

# Check if docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed"
    exit 1
fi

# Build image
echo "📦 Building image..."
docker build -t "${REGISTRY}/${IMAGE_NAME}:${VERSION}" .
docker build -t "${REGISTRY}/${IMAGE_NAME}:latest" .

# Check if docker is logged in
if ! docker info | grep -q "Username"; then
    echo "🔑 You need to login to Docker Hub first"
    echo "Running: docker login"
    docker login
fi

# Push image
echo "📤 Pushing image to Docker Hub..."
docker push "${REGISTRY}/${IMAGE_NAME}:${VERSION}"
docker push "${REGISTRY}/${IMAGE_NAME}:latest"

echo ""
echo "✅ Successfully deployed!"
echo ""
echo "Install on BlueOS:"
echo "  1. Go to BlueOS Extensions"
echo "  2. Click 'Install Extension'"
echo "  3. Enter: ${REGISTRY}/${IMAGE_NAME}:latest"
echo ""
echo "Access the app:"
echo "  http://blueos.local:9999"
echo ""
