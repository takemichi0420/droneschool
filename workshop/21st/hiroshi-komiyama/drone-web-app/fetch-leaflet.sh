#!/bin/bash
# Download and unpack Leaflet for local use (offline capability)

set -e

FRONTEND_DIR="$(dirname "$0")/frontend"

echo "Downloading Leaflet 1.9.4..."
LEAFLET_VERSION="1.9.4"
LEAFLET_URL="https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist"

mkdir -p "$FRONTEND_DIR"/leaflet/{js,css,images}

# Download CSS
curl -fsSL "${LEAFLET_URL}/leaflet.css" -o "$FRONTEND_DIR/leaflet/css/leaflet.css"
echo "✓ Downloaded leaflet.css"

# Download JS
curl -fsSL "${LEAFLET_URL}/leaflet.js" -o "$FRONTEND_DIR/leaflet/js/leaflet.js"
echo "✓ Downloaded leaflet.js"

# Download images (markers, etc.)
curl -fsSL "${LEAFLET_URL}/images/marker-icon.png" -o "$FRONTEND_DIR/leaflet/images/marker-icon.png"
curl -fsSL "${LEAFLET_URL}/images/marker-shadow.png" -o "$FRONTEND_DIR/leaflet/images/marker-shadow.png"
curl -fsSL "${LEAFLET_URL}/images/marker-icon-2x.png" -o "$FRONTEND_DIR/leaflet/images/marker-icon-2x.png"
echo "✓ Downloaded marker images"

# Link css and js files to root leaflet directory for unpkg-style access
cp "$FRONTEND_DIR/leaflet/css/leaflet.css" "$FRONTEND_DIR/leaflet/leaflet.css"
cp "$FRONTEND_DIR/leaflet/js/leaflet.js" "$FRONTEND_DIR/leaflet/leaflet.js"
echo "✓ Symlinked leaflet.css and leaflet.js to leaflet/"

echo "Leaflet locally installed at frontend/leaflet/"
