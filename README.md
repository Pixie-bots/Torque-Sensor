# Strain Gauge Based Torque Sensor

This repository contains the design report and associated resources for a strain gauge-based torque sensor developed as part of the EN2160 - Electronic Design Realization course at the University of Moratuwa. This project focuses on creating a **compact, low-cost, and wireless solution** for accurate torque measurement, suitable for various applications in academic, research, and light industrial environments.

---

## Introduction

Torque sensors are vital for monitoring and controlling rotational forces in diverse mechanical systems, from robotics and industrial automation to automotive testing and aerospace. Traditional commercial solutions can be costly or bulky. Our project addresses this by proposing a strain gauge-based torque sensor that offers an economical and untethered approach to precise torque measurement.

The system utilizes foil-type strain gauges bonded to a rotating shaft to detect minute deformations under applied torque. These mechanical deformations are then converted into electrical signals using a Wheatstone bridge configuration. These signals are subsequently amplified, digitized, and transmitted wirelessly via a microcontroller equipped with Bluetooth Low Energy (BLE) capability. A key innovation for true wireless operation is the incorporation of inductive wireless power transmission, eliminating the need for physical connections and enabling seamless use in rotating or mobile applications.

---

## Features

* **Strain Gauge Sensing:** Employs a full Wheatstone bridge configuration with two dual-grid strain gauges mounted at $\pm$45° for high sensitivity and temperature compensation.
* **Precision Signal Conditioning:** Uses an INA333 instrumentation amplifier for precise amplification and an MCP4728 DAC for digitally programmable offset adjustment.
* **High-Resolution ADC:** Integrates an HX711 24-bit Analog-to-Digital Converter, optimized for bridge-based measurements, ensuring high accuracy.
* **Wireless Data Transmission:** Features a DA14531MOD microcontroller with integrated BLE 5.1 for low-power, wireless data communication to a paired device.
* **Inductive Wireless Power:** Powers the entire system wirelessly using a Qi-compatible inductive charging setup, enabling operation on rotating shafts without physical connections.
* **Compact and Robust Design:** Designed for ease of integration with a focus on protecting internal components from environmental exposure.

---

## System Overview

The torque sensor system is engineered for high-precision measurement and wireless data transmission on a rotating shaft without physical power lines. It comprises five interconnected functional units:

### Sensing and Signal Conditioning

The core of the sensing unit is a full Wheatstone bridge, which uses two dual-grid strain gauges precisely mounted on the shaft. When torque is applied, these gauges deform, causing a minute change in their electrical resistance. This change generates a differential voltage signal, which is then fed into an INA333 instrumentation amplifier. The amplifier provides a fixed gain of 160x. To optimize the signal for the ADC, a digitally programmable offset voltage is applied via an MCP4728 DAC, compensating for baseline shifts and maximizing the ADC's dynamic range.

### Analog-to-Digital Conversion (ADC)

The amplified analog signal is then processed by the HX711, a 24-bit ADC specifically chosen for its suitability with bridge-based measurements. This ADC offers high resolution, integrated gain control, and efficient differential input handling. It samples the analog signal at either 10 Hz or 80 Hz and transmits the digital data through a simple two-wire interface.

### Wireless Communication and Processing

A compact and low-power DA14531MOD microcontroller serves as the central processing unit. It reads the digital data from the HX711 and transmits this torque data wirelessly using its integrated BLE 5.1 capability to a paired device, such as a smartphone or PC. The MCU is also responsible for managing BLE advertising, pairing processes, and error handling, while providing general-purpose input/output (GPIO) pins for system control and debugging.

### Wireless Power Supply System

To ensure untethered operation, the system incorporates a Qi-compatible inductive charging setup for power delivery. A USB-powered inverter drives a transmitting coil, generating a high-frequency AC magnetic field. This field is inductively coupled to a receiver coil located on the rotating shaft. The receiver circuit then rectifies the received AC signal using a Schottky rectifier, filters it, and regulates it into a stable 5V DC power supply for all onboard electronics.

### System Flow Summary

The entire signal and power path can be summarized as follows:

1.  Torque is applied to the shaft, resulting in mechanical strain.
2.  The strain is detected by the strain gauge Wheatstone bridge, which converts it into a differential voltage.
3.  This voltage is amplified by the INA333 and offset-adjusted by the MCP4728.
4.  The conditioned analog signal is digitized by the HX711 and sent to the DA14531 MCU.
5.  The MCU processes the data and transmits the torque information wirelessly via BLE.
6.  All electronic components on the rotating shaft are powered by a wirelessly received and rectified 5V supply.

---

## Theory & Principles

### Basic Principle of Torque Measurement via Strain Gauges

Our approach to torque measurement leverages the fundamental relationship between applied torque and the resulting shear strain in a rotating shaft. When torque is applied, the shaft undergoes torsional deformation, inducing shear strain on its surface. Strain gauges, strategically bonded to the shaft, convert this mechanical deformation into a measurable change in electrical resistance. By precisely analyzing this resistance change, the magnitude of the applied torque can be accurately determined.

### Strain Gauges

A **strain gauge** is a sensor whose electrical resistance changes proportionally with the strain it experiences. Typically, it consists of a metallic foil grid bonded to the surface being measured. As the surface deforms, the gauge's length and cross-sectional area change, leading to a corresponding alteration in its resistance. The sensitivity of a strain gauge is quantified by its **Gauge Factor (GF)**, defined as:

$$GF = \frac{\Delta R / R}{\varepsilon}$$

where $\Delta R$ is the change in resistance, $R$ is the original resistance, and $\varepsilon$ is the strain. For optimal torque measurements, strain gauges are commonly mounted at $\pm45^\circ$ angles relative to the shaft's axis to effectively capture shear strain.

### Wheatstone Bridge Configuration

Strain-induced resistance changes are typically very small, making a direct measurement challenging. Therefore, a **Wheatstone bridge** circuit is employed to detect these minute changes with high precision. This circuit consists of four resistors arranged in a diamond shape. The bridge configuration varies based on the number of active strain gauges: quarter-bridge (one active gauge), half-bridge (two active gauges), or full-bridge (four active gauges).

For this project, we opted for the **full-bridge configuration**. This setup uses four active strain gauges, with two placed in areas experiencing tensile strain and two in areas under compressive strain. This configuration offers significant advantages, including maximum sensitivity due to all four gauges contributing to the output signal, effective temperature compensation as all gauges experience similar environmental conditions, and an improved signal-to-noise ratio due to the amplified differential output. These benefits were crucial for achieving the desired accuracy and thermal stability in our torque sensor application.

### Relationship Between Torque, Strain, and Output Voltage

The relationship between torque, shear stress, and strain is fundamental to this measurement system. Shear stress ($\tau$) on the shaft is directly proportional to the applied torque ($T$), the outer radius ($r$), and inversely proportional to the polar moment of inertia ($J$):

$$\tau = \frac{T \cdot r}{J}$$

For a hollow circular shaft, the polar moment of inertia ($J$) is given by:

$$J = \frac{\pi}{2} \cdot \left( r_o^4 - r_i^4 \right)$$

where $r_o$ is the outer radius and $r_i$ is the inner radius. The shear strain ($\gamma$) is then related to the shear stress by the shear modulus ($G$) of the shaft material:

$$\gamma = \frac{\tau}{G}$$

Finally, this strain ($\varepsilon$) causes a change in the resistance of the strain gauge, which in turn alters the output voltage of the Wheatstone bridge. This output voltage is ultimately proportional to the applied torque, enabling accurate, real-time measurement.

---

## Design Considerations

### Mechanical Design

#### Overview

The mechanical design is centered on a solid cylindrical shaft housed within a protective enclosure. Torque is applied to the shaft through its exposed ends, while the central section, where the strain gauges and electronics are located, remains within the sealed housing. This design ensures that the strain gauges accurately measure the torsional strain induced by the applied torque.

#### Design Objectives

The primary goals guiding the mechanical design were to:
* **Enable Accurate Torque Transmission and Measurement:** Ensure the shaft and strain gauge mounting area are designed to produce measurable and reliable strain under expected torque loads.
* **Ensure Structural Integrity and Safety:** Select appropriate materials and dimensions to prevent failure or permanent deformation of the shaft under anticipated torque loads, incorporating a sufficient safety factor.
* **Protect Internal Components with a Robust Enclosure:** Create a sealed housing to shield the sensitive strain gauges, wiring, and electronics from environmental factors, while allowing free rotation of the shaft at both ends.
* **Support Long-Term Fatigue Resistance:** Design for durability under cyclic loading conditions, selecting materials and geometries that prevent fatigue failure over extended periods of use.
* **Facilitate Easy Manufacturing and Assembly:** Utilize standard, machinable parts and straightforward joining methods to simplify fabrication, strain gauge installation, and ongoing maintenance.

### Shaft Material Selection

#### Selection Methodology

The selection of the shaft material followed a systematic approach involving decision matrices and comprehensive property analysis. Key evaluation criteria included mechanical properties (e.g., yield strength, shear modulus), cost-effectiveness, ease of machinability, and overall suitability for reliable strain gauge application.

---

## Getting Started

This section will cover how to set up, build, and deploy the torque sensor system.

### Prerequisites

* List any software (e.g., IDEs, compilers) and hardware (e.g., programmer, specific tools) required.

### Assembly

* Detailed steps for assembling the mechanical components (shaft, housing, strain gauges).
* Steps for soldering and connecting the electronic components (PCB assembly).
* *Include an image of the assembled mechanical structure here.*
* *Include an image of the assembled PCB here.*

### Software Installation

* Instructions for flashing the firmware to the DA14531MOD microcontroller.
* Any necessary drivers or libraries for communication.

---

## Usage

This section will guide you on how to operate the torque sensor and interpret its readings.

### Calibration

* Steps for calibrating the sensor to ensure accurate torque measurements.

### Data Acquisition

* Instructions on connecting to the sensor via BLE using a smartphone app or PC software.
* Guidance on how to log and visualize the torque data.
* *Include a screenshot of the data acquisition interface/app here.*

---

## Contributing

We welcome contributions to this project! If you have suggestions for improvements, new features, or bug fixes, please feel free to:

1.  Fork the repository.
2.  Create your feature branch (`git checkout -b feature/AmazingFeature`).
3.  Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4.  Push to the branch (`git push origin feature/AmazingFeature`).
5.  Open a Pull Request.

---

## License

This project is licensed under the [Specify your license here, e.g., MIT License] - see the `LICENSE` file for details.

---

## Team Members

* Aashir M.R.M. (220004M)
* Basith M.N.A. (220071M)
* Kumarage R.V. (220343B)
* Manawadu D.N. (220380J)
* Manawadu M.D. (220381M)
* Munavvar M.A.A. (220405T)
* Ransika L.G.C. (220514C)
* Samuditha H.K.P. (220562U)
