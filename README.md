# Sentry Gun Project: Automated Target Tracking & Firing System

This repository contains the complete documentation, source code, and design files for the **Sentry Gun Project**. This system is an intelligent robotic platform designed to detect, track, and engage targets autonomously using Computer Vision and high-precision stepper motor control.

## Table of Contents
1. [Rationale of the Project](#1-rationale-of-the-project)
2. [Background and Related Previous Works](#2-background-and-related-previous-works)
3. [Overall Diagram](#3-overall-diagram)
4. [Python Program (Control & Vision)](#4-python-program-control--vision)
    - [Raspberry Pi 4 (Master Logic)](#raspberry-pi-4-master-logic)
    - [Raspberry Pi Pico (Motor Control))](#raspberry-pi-pico-motor-control)
5. [The Part Lists (BOM](#5-the-part-lists-bom)
6. [Demonstration Clip](#6-demonstration-clip)
7. [CAD Design](#7-cad-design)
8. [High-Quality Render](#8-high-quality-render)
9. [Animation](#9-animation)

---

## 1. Rationale of the Project
In environments requiring constant surveillance or rapid response—such as security zones or agricultural pest control—manual operation is often limited by human reaction time and fatigue. The **Sentry Gun Project** was born from the need for an accessible, automated solution that bridges the gap between digital detection (Computer Vision) and physical action. By automating the aiming and firing process, we eliminate human error and provide a 24/7 reliable defense or interactive system that can react to targets with mathematical precision.

## 2. Background and Related Previous Works
This project builds upon the growing "DIY Sentry" community, combining advanced kinematics with affordable hardware like the Raspberry Pi and Stepper Motors. 

**Inspiration and References:**
* **Targeting Inspiration:** The logic for rapid engagement and firing was inspired by this automated blaster system: [Watch Inspiration on Instagram](https://www.instagram.com/reel/DE1dFRIs2EO/?utm_source=ig_web_copy_link).
* **Mechanical Foundation:** The core CAD structure and gimbal mechanism were adapted and modified from the [Pan-Tilt Stepper Motor Gimbal project on Printables](https://www.printables.com/model/921719-pan-tilt-stepper-motor-gimbal), ensuring a robust and smooth 2-axis movement.

## 3. Overall Diagram
The system architecture integrates a Raspberry Pi 4 for vision processing, connected to a Raspberry Pi Pico (on the Cytron Motion 2350 PRO) via UART. The mechanical system is powered by a 24V DC supply, driving NEMA steppers through microstep drivers.

![Wiring Diagram](Wiring_Diagram.png)

## 4. Python Program (Control & Vision)

### Raspberry Pi 4 (Master Logic)
Handles object detection (OpenCV), coordinate transformation, and user commands.

📄 **[`pi4_vision.py`](pi4_vision.py)**

### Raspberry Pi Pico (Motor Control)
Handles homing, S-curve stepper motion, and firing control.

📄 **[`pico_motion.py`](pico_motion.py)**

## 5. The Part Lists (BOM)
Detailed specifications, suppliers, and pricing for all mechanical and electronic components.

🔗 **[View Bill of Materials (Google Docs)](https://docs.google.com/document/d/13ywRA99andXz3X4eBQuF-4wUD0CNCDNxtUGLMidMCd8/edit?tab=t.0)**

## 6. Demonstration Clip
See the Sentry Gun in action, tracking and engaging targets in real-time.

[![Sentry Gun Demo](https://img.youtube.com/vi/y7a5NazC8Oo/maxresdefault.jpg)](https://www.youtube.com/watch?v=y7a5NazC8Oo)

## 7. CAD Design
Complete 3D models of the Sentry Gun assembly.

**[Download CAD Files (Google Drive)](https://drive.google.com/file/d/1mrUgY9s_Yw8mawP619Sirvth3tozgA6Q/view?usp=sharing)**

## 8. High-Quality Render
Visualization of the final assembled product.

**[View Renders (Google Drive)](https://drive.google.com/file/d/17qIY9MEWtkzhQ20ECSBVelnX3ph_gVum/view?usp=sharing)**

## 9. Animation
Dynamic animation showing the range of motion and firing sequence.

**[Watch Animation (Google Drive)](https://drive.google.com/file/d/1w4ELWVbXcfN9jxWqPiFjKmMpadruqyHq/view?usp=sharing)**

---
