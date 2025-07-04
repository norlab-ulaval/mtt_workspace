# MTT-154 All-Terrain Vehicle: CAN Bus Specification

**Document Version:** 1.1 (Corrected)  
**Date:** 2025-07-03  
**Sources:** MTT-154 Owner's Manual (2024), Internal Engineering C-Code, and Patch Notes (2025-07-03)  
**Purpose:** This document provides a comprehensive specification of the CAN bus protocol used in the MTT-154 vehicle. It is intended for engineering, development, and diagnostic purposes.

## Document Change History

### Version 1.1 (2025-07-03) - Critical Patches Applied
**Based on Cédrick's email communication:**

1. **Security Switch Correction:** 
   - Original: Bit 3 (0x08) 
   - **Corrected:** Bit 7 (0x80) - CONFIRMED CORRECT

2. **Emergency Stop Patch (TEMPORARY):**
   - Light control now acts as emergency stop mechanism
   - This is a **temporary firmware behavior** - may be corrected in future versions
   - Both security switch AND light state must be operational for vehicle movement

3. **0x2FF Frame Byte Order Verification:**
   - Confirmed correct MSB-first format for both tachometer fields

### Version 1.0 (Original)
- Initial documentation based on owner's manual analysis

## 1.0 General Bus Characteristics

- **Primary Protocol:** CAN bus (Controller Area Network)
- **Bit Rate:** To Be Determined (TBD). Assumed to be 250 kbps or 500 kbps.
- **Frame Formats:**
  - **Standard CAN (11-bit identifier):** Used for all primary vehicle control and status messages.
  - **Extended CAN (29-bit identifier):** Used exclusively for communication with the battery charger.
- **Termination:** Standard 120-ohm termination resistors are assumed at each physical end of the main bus.

## 2.0 Node (Module) Identification

The following Electronic Control Units (ECUs) are present on the CAN bus.

| Node Name | Manual Reference | C-Code Reference | Description |
|-----------|------------------|------------------|-------------|
| **Main Display / Control** | A3, Diagnostic Menu | Display/Remote Control | The primary control unit. It hosts the user display, processes joystick commands, and manages overall vehicle logic. It acts as a gateway, prioritizing commands from 0x100 over 0x001. |
| **Power Controller** | Appendix A | N/A | Controls the main traction motor and provides detailed diagnostic feedback. |
| **BMS (Battery Management System)** | Diagnostic Pg. 2 | CAN_BMS_TRAME_* | Manages and reports the status of the 5.1 kWh battery pack. |
| **Joystick Receiver** | D7, B4 | CAN_WIFI_ADDRESS1 | Receives wireless signals from the remote joystick and places them on the bus. |
| **Charger** | N/A | CAN_CHARGER_* | External or internal battery charger communicating via Extended CAN. |
| **Module 300** | Diagnostic Pg. 3 | CAN_MAIN_BOARD1 | Broadcasts the Hardware and Software versions of the Main Controller. |
| **Module 301** | Diagnostic Pg. 3 | CAN_MAIN_BOARD2 | Broadcasts the Hardware and Software versions of the Battery Controller. |

## 3.0 Message Dictionary

### 3.1 Vehicle Control Frames

#### 3.1.1 Primary Joystick Control
- **CAN ID:** 0x001 (1)
- **Source:** Joystick Receiver
- **Description:** The primary frame for controlling the vehicle's movement and functions, originating from the wireless remote.

#### 3.1.2 Auxiliary/External Control
- **CAN ID:** 0x100 (256)
- **Source:** External Control System (e.g., autonomous controller)
- **Description:** This frame allows an external system to control the vehicle. When a message with this ID is present on the bus, the Main Display/Control module ignores the 0x001 joystick frame and uses this frame's data instead.
- **Frequency:** Must be streamed at a minimum of 5 Hz for basic operation. A higher rate is recommended for real-time control.

**Payload for 0x001 and 0x100 (8 Bytes):**

| Byte | C-Code #define | Parameter | Data Encoding & Notes |
|------|----------------|-----------|----------------------|
| 0 | MTT_SWITCHES_VEHICLE_TYPE | Vehicle Type | 0x00: Single Track<br>0x01: Side-by-Side Left<br>0x02: Side-by-Side Right |
| 1 | MTT_SWITCHES_GLOBAL | Global Switches | Bitmask:<br>Bit 7: Security Switch (1=Unlocked, 0=Locked)<br>Bit 6: Light (0=On, 1=Off) - Note: Inverted logic<br>Bit 5: Direction (0=Forward, 1=Reverse) - Note: Inverted logic<br>(Other bits reserved) |
| 2 | MTT_ANALOG_THROTTLE | Throttle | 0 to 230. Values above 230 are clamped. |
| 3 | MTT_ANALOG_WINCH | Winch Control | 0xe5: Winch IN<br>0x7f: Winch Neutral<br>0x18: Winch OUT |
| 4 | MTT_ANALOG_BRAKE | Brake | 0 to 255. |
| 5 | MTT_ANALOG_STEER | Steering | 0 to 255. Represents full left to full right. |
| 6 | MTT_SWITCHES_DIRECTION_MODE | Steering Mode | Bitmask:<br>Bit 0: Steering Mode (0=Open Loop, 1=Closed Loop) |
| 7 | Reserved | -- | -- |

**Note on Security:** For the vehicle to move, the Security Switch (Byte 1, Bit 7) must be set to Unlocked (1). 

**⚠️ CRITICAL TEMPORARY PATCH (2025-07-03):** According to Cédrick's email confirmation, the Light switch (Byte 1, Bit 6) currently acts as an **emergency stop mechanism** due to firmware behavior. This is a **temporary workaround** and may be corrected in future firmware versions. Both the security switch and light state must be in the correct operational state before sending motion commands.

**Patch Evolution:**
- Original: Only security switch required
- **Current (PATCH):** Security switch + light state emergency stop
- Future: Security switch only (when firmware is updated)

### 3.2 Vehicle and Module Status Frames

#### 3.2.1 Main Module Status (Tachometer & Temp)
- **CAN ID:** 0x2FF (767)
- **Source:** Main Controller
- **Description:** Provides real-time speed, distance, and temperature data from the main module.

**⚠️ CRITICAL BYTE ORDER CONFIRMATION (2025-07-03):** Per Cédrick's direct email communication, the byte order shown below is **CONFIRMED CORRECT**. This confirmation from the manufacturer supersedes any previous documentation discrepancies.

| Byte(s) | Parameter | Data Type | Description |
|---------|-----------|-----------|-------------|
| 0 | MainSensorTempA | int8_t | Main module temperature sensor A (°C). |
| 1 | MainSensorTempB | int8_t | Main module temperature sensor B (°C). |
| 2-3 | Tachimeter_Instant | uint16_t | Instantaneous speed in raw encoder ticks per second (RPS). **MSB first** (byte 2=MSB, byte 3=LSB). |
| 4-7 | Tachimeter_Cumulative | uint32_t | Cumulative distance in raw encoder ticks. Used for the odometer. **MSB first** (byte 4=MSB, byte 7=LSB). |

**Implementation Reference:** The mock server implementation has been updated to match the format confirmed by Cédrick's email communication.

#### 3.2.2 Main Controller Version
- **CAN ID:** 0x300 (768)
- **Source:** Module 300 (Main Controller)
- **Description:** Broadcasts the hardware and software version of the main controller.

| Byte(s) | Parameter | Data Type | Description |
|---------|-----------|-----------|-------------|
| 0-3 | Hardware_Revision | float | Hardware version number. MSB first. |
| 4-7 | Software_Revision | float | Software (firmware) version number. MSB first. |

#### 3.2.3 Battery Controller Version
- **CAN ID:** 0x301 (769)
- **Source:** Module 301 (Battery Controller)
- **Description:** Broadcasts the hardware and software version of the battery controller.

| Byte(s) | Parameter | Data Type | Description |
|---------|-----------|-----------|-------------|
| 0-3 | Hardware_Revision | float | Hardware version number. MSB first. |
| 4-7 | Software_Revision | float | Software (firmware) version number. MSB first. |

### 3.3 Battery Management System (BMS) Frames

#### 3.3.1 BMS - Cell Temperatures
- **CAN ID:** 0x600 (1536)
- **Source:** BMS
- **Description:** Reports the temperatures of the four main cell groups in the battery.

| Byte(s) | Parameter | Data Type | Description |
|---------|-----------|-----------|-------------|
| 0-1 | Temp1 | int16_t | Temperature of cell group 1. MSB first. |
| 2-3 | Temp2 | int16_t | Temperature of cell group 2. MSB first. |
| 4-5 | Temp3 | int16_t | Temperature of cell group 3. MSB first. |
| 6-7 | Temp4 | int16_t | Temperature of cell group 4. MSB first. |

#### 3.3.2 BMS - System Temperatures
- **CAN ID:** 0x601 (1537)
- **Source:** BMS
- **Description:** Reports ambient, MOSFET, and heating pad temperatures.

| Byte(s) | Parameter | Data Type | Description |
|---------|-----------|-----------|-------------|
| 0-1 | TempAmbiant | int16_t | Ambient temperature inside the battery. MSB first. |
| 2-3 | TempMOS | int16_t | MOSFET temperature. MSB first. |
| 4-5 | TempCustom_A | int16_t | Temperature of heating pad A. MSB first. |
| 6-7 | TempCustom_B | int16_t | Temperature of heating pad B. MSB first. |

#### 3.3.3 BMS - Core Status (SOC, V, I)
- **CAN ID:** 0x602 (1538)
- **Source:** BMS
- **Description:** The primary BMS status frame with the most critical information.

| Byte(s) | Parameter | Data Type | Description |
|---------|-----------|-----------|-------------|
| 0 | SOC | uint8_t | State of Charge (%). |
| 1-2 | Current | int16_t | Battery Current. MSB first. |
| 3-4 | Voltage | uint16_t | Battery Voltage. MSB first. |
| 5 | Heatpads_State | uint8_t | Bit 0: Pad B (0=Off, 1=On). <br>Bit 1: Pad A (0=Off, 1=On). |
| 6-7 | Charge_Time_Remaining | uint16_t | Remaining time for a full charge (minutes). MSB first. |

### 3.4 Charger Control Frames (Extended 29-bit)

#### 3.4.1 Charger Command
- **CAN ID:** 0x1806E5F4
- **Source:** Main Controller
- **Description:** Sent to the charger to command the desired charging voltage and current. Must be sent at least once per second to keep the charger active.

| Byte(s) | Parameter | Data Type | Scaling |
|---------|-----------|-----------|---------|
| 0-1 | Max_Voltage | uint16_t | Target voltage. MSB first. |
| 2-3 | Max_Current | uint16_t | Target current. MSB first. |
| 4-7 | Reserved | -- | -- |

#### 3.4.2 Charger Status Response
- **CAN ID:** 0x18FF50E5
- **Source:** Charger
- **Description:** The charger's response, confirming its configured voltage and current.

| Byte(s) | Parameter | Data Type | Scaling |
|---------|-----------|-----------|---------|
| 0-1 | Configured_Voltage | uint16_t | Confirmed voltage. MSB first. |
| 2-3 | Configured_Current | uint16_t | Confirmed current. MSB first. |
| 4-7 | Reserved | -- | -- |

## 4.0 Diagnostic Trouble Codes (DTCs)

The Power Controller reports faults via flashing LED patterns. The following table maps these patterns to structured codes.

| LED Flashes | Proposed DTC (Hex) | Description |
|-------------|-------------------|-------------|
| 1, 2 | 0xC102 | Over-voltage error. |
| 1, 3 | 0xC103 | Low-voltage error. |
| 1, 4 | 0xC104 | Over-temperature warning (Controller > 90°C). |
| 2, 2 | 0xC202 | Internal voltage fault. |
| 2, 3 | 0xC203 | Over-temperature fault (Controller > 100°C). |
| 2, 4 | 0xC204 | Throttle error at power-up (non-zero throttle). |
| 3, 1 | 0xC301 | Frequent reset fault. |
| 3, 2 | 0xC302 | Internal reset (transient fault). |
| 3, 3 | 0xC303 | Hall throttle open or short-circuit. |
| 3, 4 | 0xC304 | Non-zero throttle on direction change. |
| 4, 1 | 0xC401 | Regen or start-up over-voltage. |
| 4, 3 | 0xC403 | Motor over-temperature. |

## 5.0 Conversion Formulas and Calculations

This section provides the constants and functions necessary to convert raw encoder data into actual speed.

### 5.1 Gearing Constants

```c
// Gearing ratios and physical constants
#define MTT_GEAR1 16       // User input
#define MTT_GEAR2 36       // User input
#define MTT_GEAR3 15       // User input
#define MTT_GEAR4 32       // User input
#define MTT_GEAR_DRIVE 8   // User input
#define MTT_GEAR_TRACK 54  // Number of teeth in the caterpillar track
#define MTT_TRACK_LENGTH_CM 393 // Length of the track in cm for 1 full rotation
#define MTT_Encoder_TEET 5 // Number of teeth on the encoder wheel
#define MTT_TRACK_LENGTH_KM (MTT_TRACK_LENGTH_CM / 100000.0) // Fixed
```

### 5.2 Speed Calculation

The speed is calculated from the raw ticks per second (RPS) received in the 0x2FF frame.

**Note on Direction:** The encoder does not provide directional data (no quadrature). The direction of travel (forward/reverse) must be inferred from the last direction command sent in the control frame (ID 0x100 or 0x001, Byte 1, Bit 5).

```c
// Global variable to store the pre-calculated ratio
volatile float FinalRatio;

// Pre-calculation function (to be called once at initialization)
void RPS_to_KMh_Precalc(void)
{
    float ratio1, ratio2;
    ratio1 = ((float)MTT_GEAR2 / (float)MTT_GEAR1) * (float)MTT_Encoder_TEET;
    ratio2 = ((float)MTT_GEAR4 / (float)MTT_GEAR3) * ratio1;
    FinalRatio = (((float)MTT_GEAR_TRACK / (float)MTT_GEAR_DRIVE) * ratio2) * 2;
    return;
}

// Function to convert raw RPS to km/h
float RPS_to_KMh(float RPS)
{
    if (FinalRatio == 0.0) return 0.0; // Avoid division by zero
    return ((float)RPS / FinalRatio) * (float)MTT_TRACK_LENGTH_KM * 3600.0;
}
```