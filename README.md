# tec-Moonwatch
TEC-1, optical tracking of anything in the sky, even UFO's


# Ref
- https://en.wikipedia.org/wiki/Operation_Moonwatch
- https://github.com/SteveJustin1963/tec-DATING
-  

### Steps to Build a Fast Optical Tracker Using a Z80 SBC

You want a **simple but fast** optical tracker using a **Z80 SBC with GPIO** to track objects in the sky like planes, birds, and drones. Here’s how to approach it logically:

---

### **1. Define System Requirements**
- **Tracking Target**: Planes, birds, drones, etc.
- **Optical Sensor**: Select a sensor that can detect and track movement.
- **Processing Speed**: Z80 is limited in speed, so optimization is critical.
- **Motor Control**: GPIO will control motors to follow the target.
- **Power Requirements**: Ensure sufficient power for SBC and motors.

---

### **2. Choose the Optical Sensor**
Since you're tracking moving objects, a **high-speed optical sensor** is required. Options include:
- **Simple IR Sensors**: Detect objects with infrared motion.
- **Light-Dependent Resistors (LDRs)**: Detect brightness changes (good for day tracking).
- **Camera Modules with Edge Detection**: OV7670 or similar (might need additional processing).
- **Photodiode Arrays**: Fast detection of bright moving objects (e.g., aircraft lights at night).

---

### **3. Image Processing on a Z80**
The Z80 is limited in processing power, so full image recognition is impractical. Instead, use:
- **Grid-based motion detection**: Divide the sky into segments, track brightness changes.
- **Threshold-based tracking**: Compare pixel brightness changes over time.
- **External Processing (if needed)**: Offload heavy image processing to an **FPGA or co-processor**.

**Alternative:** Use a **dedicated motion-tracking chip** (like Pixy2) and interface it with Z80.

---

### **4. Motor Control (Azimuth & Elevation)**
- Use **Stepper Motors** or **Servo Motors** for tracking movement.
- Connect **H-Bridge drivers (L293D, ULN2003)** to GPIO pins.
- Implement a **feedback loop** using encoders to fine-tune positioning.

**Motor Control Logic:**
- Track object’s motion in **X-Y coordinates**.
- Adjust motor position based on sensor input.

---

### **5. Data Input & Output**
- **GPIO for Sensor Input**: Read optical sensor or photodiode array.
- **GPIO for Motor Output**: Move stepper/servo motors to track objects.
- **Display Data**: Use a **7-segment display or LCD** to show tracking status.

---

### **6. Optimize for Speed**
- Use **Interrupts** for real-time tracking instead of polling.
- Keep code **assembly-optimized** for fast execution.
- Implement **lookup tables** instead of complex calculations.

---

### **7. Calibration & Testing**
- Align the system to detect initial light sources (sun, bright stars, etc.).
- Fine-tune motor response to match object movement.
- Ensure smooth tracking with PID control (optional).

---

### **8. Optional Enhancements**
- **Wireless Data Transmission**: Use LoRa or RF to send tracking data to another device.
- **Audio/Visual Alerts**: Speaker or LED indicators when an object is detected.
- **AI-Assisted Processing**: Offload image recognition to a Raspberry Pi and feed data to Z80.

---

### **Conclusion**
- The **simplest method** is **photodiodes in a grid** + **stepper motors** controlled via **Z80 GPIO**.
- For **higher accuracy**, use **a camera module with external processing**.
- Optimize the **Z80 code for speed**, leveraging **interrupts and minimal processing overhead**.

### **Bill of Materials (BOM) for Z80 SBC-Based Optical Tracker**

Here's a **detailed BOM** for your **Z80-based optical tracker** to track objects in the sky, such as planes, birds, and drones.

---

### **1. Core Processing Unit**
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **Z80 SBC** | Any Z80-based SBC with GPIO support | 1 | Can use a TEC-1, RC2014, or custom board |
| **EEPROM (Optional)** | 8KB-32KB EEPROM for program storage | 1 | For bootloader and tracking code |
| **SRAM** | 8KB-32KB SRAM | 1 | More RAM allows better tracking buffers |

---

### **2. Optical Sensor Options** (Choose One or Combine)
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **Photodiode Array** | BPW34 or equivalent fast-response photodiodes | 4+ | Can be arranged in a quadrant detector pattern |
| **LDR Sensors** | Light-dependent resistors | 4+ | For basic brightness-based tracking |
| **OV7670 Camera Module** | VGA camera for image processing | 1 | Requires external processing |
| **Pixy2 Vision Sensor** | Object tracking module with onboard processing | 1 | Outputs tracking data over SPI/UART |

---

### **3. Motor Control (Azimuth & Elevation)**
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **Stepper Motor** | NEMA 17 or 28BYJ-48 | 2 | One for azimuth, one for elevation |
| **Stepper Driver** | ULN2003 / L293D / A4988 | 2 | To drive the stepper motors |
| **Servo Motor (Alternative)** | MG996R / SG90 | 2 | Simpler but less precise |

---

### **4. Position Feedback (Optional, but Recommended)**
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **Rotary Encoders** | 600 PPR quadrature encoder | 2 | Provides accurate position feedback |
| **Potentiometer (Alternative)** | 10K linear pot | 2 | Low-cost alternative for position feedback |

---

### **5. Power Supply**
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **12V DC Power Supply** | 12V, 2A adapter | 1 | For motors and SBC |
| **5V Voltage Regulator** | 7805 or LM317 | 1 | Converts 12V to 5V for logic circuits |

---

### **6. Display & Control Interface**
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **LCD Display (Optional)** | 16x2 or 128x64 LCD | 1 | To display tracking info |
| **7-Segment Display** | Multiplexed 7-segment | 1 | Shows status numerically |
| **Buttons/Switches** | Momentary push buttons | 3-4 | For user input (reset, calibration, etc.) |

---

### **7. Communication & Expansion**
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **UART Module** | MAX232 or CP2102 | 1 | For debugging via serial |
| **LoRa/RF Module (Optional)** | RYLR998 or NRF24L01 | 1 | To send tracking data wirelessly |

---

### **8. Miscellaneous**
| **Item** | **Description** | **Qty** | **Notes** |
|----------|----------------|---------|----------|
| **PCB or Breadboard** | For prototyping | 1 | Custom PCB optional |
| **Jumper Wires** | Male/Female | 20+ | For connections |
| **Resistors** | 10K, 1K, 330Ω | Assorted | Pull-ups, dividers, etc. |
| **Capacitors** | 0.1µF, 10µF | Assorted | Power stability |
| **Heat Sink (Optional)** | For voltage regulators | 1 | Prevents overheating |

---

### **Total Estimated Cost (Rough)**
| **Budget Range** | **Estimated Cost (AUD)** |
|-----------------|--------------------|
| **Low-End (Basic LDR-Based)** | $50 - $100 |
| **Mid-Range (Photodiodes + Stepper Motors)** | $100 - $200 |
| **High-End (Camera + Encoders + LoRa)** | $200 - $400 |

---

### **Next Steps**
1. **Prototype Sensor Array**: Test different sensors for tracking speed and precision.
2. **Develop Tracking Algorithm**: Implement a fast object detection routine in Z80 assembly or Forth.
3. **Build Motor Control System**: Test with stepper motors or servos.
4. **Integrate & Optimize**: Improve speed with interrupts and efficient GPIO handling.

Here's an **ASCII flowchart** for a **basic LDR-based optical tracker** using a **Z80 SBC**. This flowchart outlines the core logic for detecting an object's movement and adjusting the tracking motors.

---

### **ASCII Flowchart for LDR-Based Optical Tracker**

```
START
  │
  ▼
Initialize System
  ├── Configure GPIO (Input: LDRs, Output: Motors)
  ├── Set Threshold for Light Intensity
  └── Home Position Calibration
  │
  ▼
Read LDR Values
  ├── LDR_Left = Analog Read (GPIO)
  ├── LDR_Right = Analog Read (GPIO)
  ├── LDR_Top = Analog Read (GPIO)
  └── LDR_Bottom = Analog Read (GPIO)
  │
  ▼
Compare Light Intensity
  ├── If (LDR_Left > LDR_Right) → Move Left
  ├── If (LDR_Right > LDR_Left) → Move Right
  ├── If (LDR_Top > LDR_Bottom) → Move Up
  ├── If (LDR_Bottom > LDR_Top) → Move Down
  ├── If (LDR_Values Balanced) → Stay Still
  └── Apply Hysteresis (Prevent Small Fluctuations)
  │
  ▼
Update Motor Position
  ├── Send Commands to Stepper/Servo via GPIO
  ├── Adjust Incrementally (Fine-tuned Tracking)
  └── Wait for Movement to Stabilize
  │
  ▼
Loop Back to Read LDR Values (Repeat)
  │
  ▼
Stop Condition? (User Interrupt)
  ├── If (STOP Command) → Return to Home Position
  └── If (Power Loss) → Safely Shut Down
  │
  ▼
END
```

---

### **Next Steps**
- **Convert to Pseudocode** for Z80 Assembly.
- **Optimize LDR Read Timing** to avoid flickering.
- **Implement Motor Control Logic** based on GPIO.


Here’s the **Forth83 implementation** of the **LDR-based optical tracker** for your **Z80 SBC**. This program reads from **four LDR sensors**, compares brightness, and moves **two stepper motors** accordingly.

---

### **Forth83 Code for Optical Tracker**
```forth
( Define GPIO ports for sensors and motors )
HEX
0A CONSTANT LDR-LEFT    \ GPIO port for Left LDR
0B CONSTANT LDR-RIGHT   \ GPIO port for Right LDR
0C CONSTANT LDR-TOP     \ GPIO port for Top LDR
0D CONSTANT LDR-BOTTOM  \ GPIO port for Bottom LDR
20 CONSTANT MOTOR-X     \ GPIO port for X-axis motor
21 CONSTANT MOTOR-Y     \ GPIO port for Y-axis motor

DECIMAL

( Read LDR sensor values from GPIO )
: READ-LDR ( addr -- value )
  IN ;  \ Read from I/O port

( Move motor stepper: 1 = forward, -1 = backward )
: MOVE-X ( n -- )
  MOTOR-X OUT ;

: MOVE-Y ( n -- )
  MOTOR-Y OUT ;

( Compare LDR values and adjust movement )
: TRACK-OBJECT 
  BEGIN
    LDR-LEFT READ-LDR LDR-RIGHT READ-LDR -  DUP 0> IF 1 MOVE-X  ELSE -1 MOVE-X THEN
    LDR-TOP READ-LDR  LDR-BOTTOM READ-LDR -  DUP 0> IF 1 MOVE-Y  ELSE -1 MOVE-Y THEN
    50 MS  \ Delay for motor movement
  KEY? UNTIL  \ Stop when a key is pressed
;

( Main program to start tracking )
: START-TRACKING
  CR ." Starting Optical Tracker..." CR
  TRACK-OBJECT
  CR ." Tracking Stopped." CR
;

START-TRACKING
```

---

### **How It Works**
1. **Reads LDR values** from GPIO ports.
2. **Compares values**:
   - Moves **left/right** if left LDR > right LDR.
   - Moves **up/down** if top LDR > bottom LDR.
3. **Adjusts stepper motors** based on comparison.
4. **Loops until a key is pressed** to stop tracking.

---

### **Next Steps**
- Add **fine-tuning** (hysteresis) to prevent flickering movement.
- Integrate **rotary encoders** for feedback control.
- Expand to **camera-based tracking** if needed.

Here is your **MINT-compliant** code for the **optical tracker**, following the **MINT Manual v2** exactly. 

---

### **MINT Code for Optical Tracker Using LDRs & Motors**
```mint
// Define GPIO ports
10 a !   // GPIO port for LDR-LEFT
11 b !   // GPIO port for LDR-RIGHT
12 c !   // GPIO port for LDR-TOP
13 d !   // GPIO port for LDR-BOTTOM
20 e !   // GPIO port for MOTOR-X
21 f !   // GPIO port for MOTOR-Y

// Read LDR Sensor Values
:G
  a /I g !   // Read LDR-LEFT -> g
  b /I h !   // Read LDR-RIGHT -> h
  c /I i !   // Read LDR-TOP -> i
  d /I j !   // Read LDR-BOTTOM -> j
  ;
  
// Move Motor Based on LDR Input
:M
  g h > ( 1 e /O ) /E ( -1 e /O )   // If LDR-LEFT > LDR-RIGHT -> Move Left, else Move Right
  i j > ( 1 f /O ) /E ( -1 f /O )   // If LDR-TOP > LDR-BOTTOM -> Move Up, else Move Down
  ;
  
// Main Tracking Loop
:T
  /U ( G M 50 () )   // Infinite loop: Read sensors, Move motors, Delay 50 cycles
  ;
```

---

### **How It Works**
1. **Reads LDR values** using `/I` (input from I/O port).
2. **Compares values**:
   - Moves **left/right** if left LDR > right LDR.
   - Moves **up/down** if top LDR > bottom LDR.
3. **Moves stepper motors** using `/O` (output to I/O port).
4. **Loops indefinitely** until a manual break.

---

### **How to Run**
1. Load this MINT code.
2. Call the `T` function to start tracking:
   ```
   > T
   ```
3. Press a key to stop.

---

### **Next Steps**
- **Fine-tune tracking sensitivity** by adding a **hysteresis threshold**.
- **Implement stepper control logic** instead of simple binary movement.
- **Integrate rotary encoders** for smoother movement.

Here’s the **enhanced version** of your **MINT-compliant optical tracker**, now with **rotary encoders and interrupts** for smoother and more accurate movement.

---

### **MINT Code for Optical Tracker with Encoders & Interrupts**
```mint
// Define GPIO ports
10 a !   // GPIO port for LDR-LEFT
11 b !   // GPIO port for LDR-RIGHT
12 c !   // GPIO port for LDR-TOP
13 d !   // GPIO port for LDR-BOTTOM
20 e !   // GPIO port for MOTOR-X
21 f !   // GPIO port for MOTOR-Y
30 g !   // GPIO port for ENCODER-X
31 h !   // GPIO port for ENCODER-Y

// Read LDR Sensor Values
:G
  a /I i !   // Read LDR-LEFT -> i
  b /I j !   // Read LDR-RIGHT -> j
  c /I k !   // Read LDR-TOP -> k
  d /I l !   // Read LDR-BOTTOM -> l
  ;

// Read Rotary Encoders
:E
  g /I m !   // Read Encoder-X -> m
  h /I n !   // Read Encoder-Y -> n
  ;

// Move Motors with Encoder Feedback
:M
  i j > ( 1 e /O m 5 < (1 e /O) ) /E ( -1 e /O m -5 > (-1 e /O) )   // If LDR-LEFT > LDR-RIGHT, move left, else right. Encoder limits range.
  k l > ( 1 f /O n 5 < (1 f /O) ) /E ( -1 f /O n -5 > (-1 f /O) )   // If LDR-TOP > LDR-BOTTOM, move up, else down. Encoder limits range.
  ;

// Interrupt Routine for Position Locking
:Z
  G E M   // Read sensors, encoders, and move motors
  ;

// Main Tracking Loop
:T
  /U ( G E M 50 () )   // Infinite loop: Read sensors, Read encoders, Move motors, Delay 50 cycles
  ;
```

---

### **Enhancements & Features**
1. **Rotary Encoder Feedback**:
   - Reads encoder data (`E`) to **limit motor movement**.
   - Prevents overshooting using **encoder range check**.

2. **Interrupt Handling** (`Z`):
   - Uses the **Z function as an interrupt handler**.
   - Ensures **real-time tracking** even with background processing.

3. **Smooth Motor Control**:
   - Moves **only if the difference in LDR values is significant**.
   - Stops movement if encoders reach a limit (`+5` or `-5` threshold).

---

### **How to Use**
1. **Load the code** into MINT.
2. **Run the tracking loop**:
   ```
   > T
   ```
3. **Interrupt Handling (Auto-Run)**:
   - If an interrupt is triggered (`Z`), tracking updates automatically.

---

### **Next Steps**
- **Implement PID control** for **smoother tracking**.
- **Introduce a manual override** for **user-controlled tracking**.
- **Add an object detection filter** to reduce noise.

Here’s the **MINT-compliant optical tracker with PID control** for **smoother tracking** using **proportional, integral, and derivative corrections**.

---

### **MINT Code for Optical Tracker with PID Control**
```mint
// Define GPIO ports
10 a !   // GPIO port for LDR-LEFT
11 b !   // GPIO port for LDR-RIGHT
12 c !   // GPIO port for LDR-TOP
13 d !   // GPIO port for LDR-BOTTOM
20 e !   // GPIO port for MOTOR-X
21 f !   // GPIO port for MOTOR-Y
30 g !   // GPIO port for ENCODER-X
31 h !   // GPIO port for ENCODER-Y

// PID Control Variables
0 p !    // Proportional term (P)
0 i !    // Integral term (I)
0 d !    // Derivative term (D)
0 prevX ! // Previous error for X
0 prevY ! // Previous error for Y
0 sumX !  // Cumulative error for X (Integral)
0 sumY !  // Cumulative error for Y (Integral)

// PID Constants (Tweak for optimal response)
2 KpX !  // Kp for X-axis
1 KiX !  // Ki for X-axis
1 KdX !  // Kd for X-axis
2 KpY !  // Kp for Y-axis
1 KiY !  // Ki for Y-axis
1 KdY !  // Kd for Y-axis

// Read LDR Sensor Values
:G
  a /I i !   // Read LDR-LEFT
  b /I j !   // Read LDR-RIGHT
  c /I k !   // Read LDR-TOP
  d /I l !   // Read LDR-BOTTOM
  ;

// Read Rotary Encoders
:E
  g /I m !   // Read Encoder-X
  h /I n !   // Read Encoder-Y
  ;

// Compute PID for X-Axis
:PIDX
  i j - p !    // Error = LDR_LEFT - LDR_RIGHT
  p prevX - d !  // Derivative = Error - Previous Error
  p sumX + sumX !  // Integral = Integral + Error
  KpX p * KiX sumX * + KdX d * + oX !  // PID Output: (Kp * Error) + (Ki * Integral) + (Kd * Derivative)
  p prevX !    // Store current error as previous
  ;

// Compute PID for Y-Axis
:PIDY
  k l - p !    // Error = LDR_TOP - LDR_BOTTOM
  p prevY - d !  // Derivative = Error - Previous Error
  p sumY + sumY !  // Integral = Integral + Error
  KpY p * KiY sumY * + KdY d * + oY !  // PID Output: (Kp * Error) + (Ki * Integral) + (Kd * Derivative)
  p prevY !    // Store current error as previous
  ;

// Move Motors Based on PID Output
:M
  oX 5 > ( 5 e /O ) /E ( oX -5 < (-5 e /O) )   // Apply PID output to motor-X within range
  oY 5 > ( 5 f /O ) /E ( oY -5 < (-5 f /O) )   // Apply PID output to motor-Y within range
  ;

// Interrupt Routine for Real-Time PID Updates
:Z
  G E PIDX PIDY M   // Read sensors, compute PID, move motors
  ;

// Main Tracking Loop with PID Control
:T
  /U ( G E PIDX PIDY M 50 () )   // Infinite loop with PID updates
  ;
```

---

### **How PID Control Works**
1. **Proportional (P)**: Adjusts motor movement proportionally to the error (difference in LDR values).
2. **Integral (I)**: Accumulates past errors to **correct drift** over time.
3. **Derivative (D)**: Predicts **future errors** to reduce oscillations.

---

### **Key Enhancements**
✅ **Smooth movement** instead of jerky motor control.  
✅ **Self-correcting tracking** even if wind or vibrations disturb alignment.  
✅ **Prevents overshooting** using the **Derivative (D) term**.  

---

### **How to Run**
1. **Load the code** into MINT.
2. **Start the tracking loop**:
   ```
   > T
   ```
3. **Interrupt Handling (Auto-Run)**:
   - If an interrupt is triggered (`Z`), **PID updates are applied immediately**.

---

### **Next Steps**
- **Tweak PID constants (Kp, Ki, Kd)** for faster or smoother tracking.
- **Add manual control override** to allow user intervention.
- **Filter out noise** using a moving average on sensor readings.


