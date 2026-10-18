# Requirements

## Overview
A web-based controller for MAVLink-compatible drones.

## Goals
- Browser-based control with FastAPI and WebSocket
- Real-time telemetry display
- Simple map-based visualization

## Functional Requirements
- Connect to MAVLink vehicle
- Arm / disarm
- Takeoff / land
- GoTo target
- Change flight mode
- Relative movement using heading-based velocity commands

## Non-Functional Requirements
- Works locally on Python 3.7+
- No build tools required for the frontend
- Responsive UI for desktop and mobile
