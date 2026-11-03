MAE 106 Pneumatic Competition Robot

Overview

This repository contains the embedded control firmware for a pneumatic tricycle robot designed and built for the UC Irvine MAE 106 (Mechatronics) competition. The robot utilizes a rack-and-pinion transmission driven by a pneumatic cylinder and relies on real-time sensor feedback to navigate a structured course quickly and safely.

**Hardware Architecture**

Microcontroller: Arduino Uno

Propulsion: MAL 16-75 pneumatic cylinder (25 psi) driving dual offset racks and one-way bearings.

Steering: Custom 3D-printed kingpins, linkages, and push rods actuated by a standard Servo.

Sensors:

  Reed Switch (for distance/tick tracking)

  LIS3MDL Magnetometer (for heading detection)

  LSM6 IMU

**Software & Control Strategy**

The firmware is written in C++ and focuses on dynamic speed adjustment and precise navigation:

Interrupt Service Routine (ISR): A custom ISR handles the reed switch feedback to track wheel rotations without missing a single tick during high-speed operation.

Dynamic Propulsion: The control logic processes filtered magnetometer data (20% alpha) to calculate heading errors. The system dynamically adjusts the solenoid valve timing (reducing firing frequency from 180ms to 300ms during turns) to prevent the tricycle chassis from tipping.

Optimized Duty Cycle: Iterative impulse response testing determined that a 150ms solenoid valve duty cycle maximizes straight-line velocity while preserving the finite onboard air supply (stored in a scooter tire).

**Team (Group 13)**

Tyler Rivas: Controls Lead (Embedded firmware, ISR, steering logic, dynamic propulsion)

Brandon Lew: Mechanical Lead

Ethan Ha: Electrical Lead
