# Wind Tunnel Speed Control using PID

This project involves controlling the speed of a wind tunnel fan using a PID controller. The system uses environmental data from multiple sensors to compute airspeed and adjusts the fan speed accordingly.

The PID gains were discoveres by applying the Ziegler-Nichols method as a ruler.

## Sensors and Measurements

The project utilizes the following sensors:
- **HDC1080**: Measures temperature and humidity.
- **BME280**: Measures barometric pressure.
- **MPXV7002DP**: Measures differential air pressure.

## Airspeed and Air Density Calculation

The airspeed is derived using the Pitot tube formula, which relates the air pressure difference to velocity. The air density is computed using environmental measurements:

1. **Temperature in Kelvin**:
   $$T_k = T_{\text{Celsius}} + 273.15$$

2. **Saturation vapor pressure**:
   $$P_{\text{sat}} = 0.61078 \cdot \exp\left(\frac{17.27 \cdot T_{\text{Celsius}}}{T_{\text{Celsius}} + 237.3}\right)$$

3. **Water vapor pressure**:
   $$P_v = P_{\text{sat}} \cdot \frac{\text{Humidity}}{100.0}$$

4. **Dry air pressure**:
   $$P_d = P_{\text{atmospheric}} - P_v$$

5. **Air density**:
   $$\rho = \frac{P_d}{287.058 \cdot T_k} + \frac{P_v}{461.495 \cdot T_k}$$

6. **Airspeed**:
   $$v = \sqrt{\frac{2 \cdot P_{\text{differential}}}{\rho}}$$



## PID Control

The system employs a PID controller to maintain the target airspeed by adjusting the fan's speed.

### PID Formula
$$\text{PID output} = K_p \cdot e + K_i \int e \, dt + K_d \frac{de}{dt}$$

- **Proportional (P)**: $`e = v_{\text{target}} - v_{\text{measured'}}`$
- **Integral (I)**: Accumulated error over time.
- **Derivative (D)**: Rate of change of error.
#### Note:
The error in the code itself is multiplied by a air speed to RPM constant which is unique to my project, this is needed to calculate the correct PWM duty cycle.


## Code Logic

### Initialization
- Configure sensors (HDC1080 and BME280) using I²C.
- Initialize moving average arrays for pressure and RPM readings.
- Set up PWM for fan speed control.
- Attach an interrupt for tachometer readings.

### Main Functions
#### `updateData()`
Reads data from sensors and calculates:
- Air density ($`\rho`$)
- Airspeed ($`v`$)

#### `tacometroInterrupt()`
Triggered on tachometer pulses:
- Calculates RPM.
- Updates moving averages for RPM and pressure.
- Computes airspeed and PID output.
- Adjusts fan speed via PWM.

#### `loop()`
Continuously prints real-time data for debugging.


### Hardware Configuration
- **Analog Pin (1)**: Reads differential pressure sensor.
- **Tachometer Pin (15)**: Measures fan RPM.
- **PWM Pin (16)**: Controls fan speed.
- **SDA Pin (12), SCL Pin (11)**: I²C communication with HDC1080 and BME280.



## Libraries Used
- [ClosedCube_HDC1080](https://github.com/closedcube/ClosedCube_HDC1080)
- [BME280](https://github.com/finitespace/BME280)


## Setup and Usage
1. Connect the sensors and configure pins as per the hardware setup.
2. Upload the code to an Arduino or ESP32 board.
3. Adjust PID constants (`K_p`, `K_i`, `K_d`) for your wind tunnel.
4. Monitor serial output for real-time airspeed and PID performance.

##Images

Here are photos of the wind tunnel setup and key components:
   Body:
      ![body](https://github.com/FuscaoPreto/Wind_Tunel_PID/blob/main/photos/body.jpg)
   Body (Alternate View):
      ![body vertical](https://github.com/FuscaoPreto/Wind_Tunel_PID/blob/main/photos/body_2.jpg)
   Mouth (Pitot Tube View):
      ![mouth](https://github.com/FuscaoPreto/Wind_Tunel_PID/blob/main/photos/mouth.jpg)
