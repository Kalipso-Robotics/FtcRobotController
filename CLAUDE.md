# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FTC (FIRST Tech Challenge) robot controller project for the INTO THE DEEP (2024-2025) season. It's based on the official FTC SDK v10.1 and contains custom robot code for "Kalipso Robotics" team.

## Build System

This is an Android project using Gradle. The main modules are:
- `FtcRobotController`: Core FTC SDK module (rarely modified)
- `TeamCode`: Custom robot code (main development area)

### Common Commands

Build the project:
```bash
./gradlew build
```

Build release APK:
```bash
./gradlew assembleRelease
```

Clean build:
```bash
./gradlew clean
```

Run tests:
```bash
./gradlew test
```

Note: If you encounter Java version errors, ensure you're using Java 11 as specified in build.common.gradle.

## Code Architecture

### Core Systems

The robot code is organized under `TeamCode/src/main/java/com/kalipsorobotics/` with these key systems:

**Localization & Navigation:**
- `localization/Odometry.java`: Main odometry system using wheel encoders, IMU, and SparkFun OTOS sensor
- `localization/SensorFusion.java`: Combines multiple sensor inputs using Kalman filtering
- `navigation/PurePursuitAction.java`: Path following using pure pursuit algorithm
- `navigation/AdaptivePurePursuitAction.java`: Advanced path following with dynamic lookahead

**Hardware Modules:**
- `modules/DriveTrain.java`: Singleton drivetrain control with multiple odometry sensors
- `modules/IMUModule.java`: IMU integration
- `modules/GoBildaOdoModule.java`: Encoder-based odometry
- `modules/GoBildaPinpointDriver.java`: GoBilda Pinpoint odometry sensor driver

**Action System:**
- `actions/actionUtilities/Action.java`: Base class for robot actions
- `actions/actionUtilities/KActionSet.java`: Action sequencing and parallel execution
- `actions/drivetrain/`: Drive-related actions

**Math & Utilities:**
- `math/`: Position, Vector, Path, and mathematical utility classes
- `PID/`: PID controller implementation
- `utilities/`: Shared utilities including gamepad handling and file I/O

**Computer Vision:**
- `tensorflow/`: TensorFlow Lite integration for object detection
- Uses `robotv2_model.tflite` for robot detection

### Key Design Patterns

1. **Singleton Pattern**: DriveTrain and Odometry use singleton pattern for global access
2. **Action Pattern**: Robot behaviors are implemented as Actions that can be sequenced
3. **Sensor Fusion**: Multiple odometry sources are combined for improved accuracy
4. **Modular Hardware**: Hardware components are abstracted into modules

### Important Constants

Located in various classes:
- `Odometry.java`: Track width (297mm), back distance offset (-70mm)
- `PurePursuitAction.java`: PID gains for different speed modes
- Encoder tick calculations in `CalculateTickPer.java`

### Testing

Test OpModes are located in:
- `test/localization/`: Odometry and path following tests
- `actions/checkStuck/checkStuck/`: Robot stuck detection tests

### Dependencies

Key external libraries (from TeamCode/build.gradle):
- EasyOpenCV for computer vision
- FTC Dashboard for debugging
- LiteRT for TensorFlow Lite
- JUnit for testing

## Development Notes

- Main robot code goes in TeamCode module
- Hardware configuration should match names used in DriveTrain.java
- Odometry requires calibration of encoder positions and track width
- Pure pursuit paths are defined as lists of Position objects
- Actions can be chained using KActionSet for complex autonomous routines