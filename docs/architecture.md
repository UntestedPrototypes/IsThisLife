# ARCHITECTURE

Below is a description of all electronics used in the project and how they interact on a high-level.

[Overview of electronical components] (TO BE ADDED)

[Program flow/interaction map of Ball-Bots, Controller and Python script] (TO BE ADDED)

## COMMS

Each robot has a ESP32 insides which connect over ESP_NOW to the main ESP32 Controller which plugs into a laptop running a python script. All Ball-Bots connect to as peers to the controller.

```
Python (Laptop) <-> Controller (ESP32) <---> Robots (ESP32)
```

### Packets

To make communication easy and extendable in the future I decided on using packet structure. There are multiple types of packets which can be send. Each contain different types of data depending on it use. (see packet.h)

[Overview of packets and types]

### Safety Heartbeat

I want to highlight the heartbeat in the packets. This is the most important safety feature of all. This is used by each Ball-Bot to ensure it gets new data. If something goes wrong this ensures the Ball-Bot will engage it's emergency stop. This can be reset by sending the correct E-stop release packet.

**These packets are not optimized yet** and all over the place at the momement because of development more and more is added. Once development reaches a balance packaging **should be revisted and clean up!**

## MOTORS & DRIVERS

The main motor is controlled by a Electronics Speed Controller (ESC) for RC Car which talks the esp32 over a PWM signal where 1500us is neutral, 1000us is back and 2000us is forward. If this sound unfimiliar to you use GPT to walk you through the code.

### Driver settings

Please read the manual before changing settings. KNOW the dangers in changing these!!!
The following settings should be programmed on the ESC (QuicRun WP10BL120 G2 Brushless ESC 120A 2-4S LiPo)


| Item | Value |
| :----- | ------- |
| 1    | 3     |
| 2    | 6     |
| 3    | 1     |
| 4    | 1     |
| 5    | 3     |
| 6    | 4     |
| 7    | 1     |
| 8    | 4     |
| 9    | 1     |
| 10   | 1     |

[ADD example code]

The secondary axis is managed by two servos, which will be replaced by Servo with encoder on a serial bus. This is connected to the second serial bus on the ESP32. To see how this is managed in more detail checkout the code or the wiki from the servos (https://www.waveshare.com/wiki/ST3215_Servo).

[ADD Example code]

## SENSORS

### 9-DOF + Temp Sensor (BNO055)

The BNO055 is used to measure the angle, gravity vector and temperature of the system.

The BNO055 can output the following sensor data:

* **Absolute Orientation** (Euler Vector, 100Hz)
  Three axis orientation data based on a 360° sphere
* **Absolute Orientation** (Quaterion, 100Hz)
  Four point quaternion output for more accurate data manipulation
* **Angular Velocity Vector** (100Hz)
  Three axis of 'rotation speed' in rad/s
* **Acceleration Vector** (100Hz)
  Three axis of acceleration (gravity + linear motion) in m/s^2
* **Magnetic Field Strength Vector** (20Hz)
  Three axis of magnetic field sensing in micro Tesla (uT)
* **Linear Acceleration Vector** (100Hz)
  Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
* **Gravity Vector** (100Hz)
  Three axis of gravitational acceleration (minus any movement) in m/s^2
* **Temperature** (1Hz)
  Ambient temperature in degrees celsius

How this would look like in code for RAW data

```
// Possible vector values can be:
// - VECTOR_ACCELEROMETER - m/s^2
// - VECTOR_MAGNETOMETER  - uT
// - VECTOR_GYROSCOPE     - rad/s
// - VECTOR_EULER         - degrees
// - VECTOR_LINEARACCEL   - m/s^2
// - VECTOR_GRAVITY       - m/s^2
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

// Get ambient temperature
int8_t temp = bno.getTemp();

// Get Quaternion
imu::Quaternion quat = bno.getQuat();
```

*(Source ->  https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor?embeds=allow)*

### Voltage Sensor (INA219)

We use a regular INA219 sensor to measure the voltage from the batteries. This is used to warn if the battery get too low and shutdown the motors to prevent battery damage.

### Adafruit Unified Sensor Drive

Some sensors support AUSD. Using this system allows abstract the need to process raw data. Instead if allows me to focus on using that data while also being easy to replace with new sensor without changing much in the code if need be.

*(Source -> https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work)*
