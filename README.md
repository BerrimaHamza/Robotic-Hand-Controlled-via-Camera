# Robotic Hand Controlled via Camera

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Setup and Installation](#setup-and-installation)
- [Usage](#usage)
- [Acknowledgments](#acknowledgments)

## Introduction

This project involves the creation of a robotic hand controlled via a camera using STM32 microcontroller and Python. The 3D design of the robotic hand is created using Fusion 360. The aim is to develop a cost-effective and efficient robotic hand that can be used for various applications, including educational purposes, research, and hobby projects.

## Features

- **Camera Control:** The robotic hand is controlled using a camera to capture movements.
- **STM32 Microcontroller:** Utilizes the STM32 microcontroller for processing and control.
- **Python Integration:** Python is used for image processing and sending commands to the microcontroller.
- **3D Design:** The mechanical parts of the hand are designed using Fusion 360.

## Hardware Requirements

- STM32 Microcontroller(STM32 H7A3ZI-Q)
- Camera module
- Servo motors
- 3D printed parts (designed in Fusion 360)
- Connecting wires and power supply

## Software Requirements

- STM32CubeIDE (or equivalent for STM32 development)
- Python 3.11 or newer
- OpenCV library for Python
- Fusion 360 for 3D design

## Setup and Installation

### Hardware Setup

1. Assemble the 3D printed parts of the robotic hand.
2. Attach the servo motors to the joints of the hand.
3. Connect the servo motors to the STM32 microcontroller.
4. Set up the camera module and connect it to your development PC.

### Software Setup

1. Clone this repository:
    ```sh
    git clone https://github.com/yourusername/robotic-hand-controlled-via-camera.git
    ```
2. Install Python dependencies:
    ```sh
    pip install opencv-python
    pip install mediapipe
    ```
3. Open the STM32CubeIDE and load the provided project files.
4. Upload the firmware to the STM32 microcontroller.

## Usage

1. Run the Python script to start the camera and capture movements:
    ```sh
    python RoboticHand.py
    ```
2. The script will process the camera input and send commands to the STM32 microcontroller to control the robotic hand.

## Acknowledgments

- Special thanks to [Hamza Berrima], [Houssem dakhli], [Najeh Ghanmi], [Amenallah Aboulkacem] for the initial development of this project.
