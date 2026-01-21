# Hardware Overview


![](assets/20260114_162203_Reference_1.jpg)

![](assets/20260114_162203_Reference_2.jpg)

This list contains relevant components for


| Part Name                   | Specifics                                              |
| :---------------------------- | -------------------------------------------------------- |
| Main Axis<br />             | 10mm aluminum rod                                      |
| Main body                   | Printed in PLA                                         |
| Secondary Axis              | Printed in PLA                                         |
| Electronic Speed Controller | QuicRun WP10BL120 G2 Brushless ESC 120A 2-4S LiPo      |
| Main motor (Main Axis)      | Motor 5065 270kv (should be 140kv)                     |
| Servos (Secondary Axis)     | TD-8125MG Waterproof Digital Servo - 25kg - Continuous |
| Power Supply - Servos       | XL4016E1 Step-Down Buck Converter                      |
| Batteries                   | 4S Lipo Battery 14.8V 120C 6500mAh with XT90           |
| MCU                         | ESP32-WROOM                                            |
|                             |                                                        |


## Hardware Limitations

1. **DO NOT TWIST SECONDARY AXIS MORE THAN 180 degrees**
2. **NEVER PLUG IN BATTERIES THAT DO NOT HAVE THE SAME VOLTAGES**


## Electronics Limitations

1. Never let the batteries go below 3.7v or above 4.2v (14.8V-16.8)
2. Never let the temperature sensor reach 50 degrees

## Arrival

1. You only need to cut the white zip ties to control the robot, the black ones are used so that cables don't drag thru the shell
2. Check battery levels for all batteries and charge to same voltage (16.4V should be nice)
3. Check jumper wires. they are either soldered or hot-glued so nothing should happen but if you see any loose wires contact me (see additional notes)

## How to use-Software side

1. Generate a Wifi network (I was using my phone hotspot)
    - WIFI_SSID = "isthislife?";
    - WIFI_PASS = "Admin1234";
2. Connect your computer to the Wifi
3. Plug any controller
3. Run Controller.py
4. Start robot with power button
5. Close sphere
6. Use left joystick to move, it is fairly sensitive so be careful

## Communication protocol:
// ================= WIFI / UDP =================
const char* WIFI_SSID = "isthislife?";
const char* WIFI_PASS = "Admin1234";
const uint16_t UDP_PORT = 4210;

Message: [RecipientID]+[message]
Robot moves expecting posxy [x,y] ->x is negative from the face that includes the ESP32
IDs: Laptop-0
Robot sent-1

Example message to control robot: 10.3,0.5

- udplisten.py allows you to just listen to messages in case you want to run it on another computer, but you can also just donwload any phone up and will be the same

## Additional Notes:
 - Controller.py will give you status info, temperature and battery voltage
 - I will roll some updates as I improve the code, in another branch I'm currently developing sensing with the IMUs which should make it nicer
 - For firmware update, remove ESP32 from base, connect to computer and flash
 - Any questions contact me. Marcos, Whatsapp: +34 616 817 686 email: marcos.merinofrancos@gmail.com 