
# Mower 8.0
## Introduction
Hi, I'm Tony, this is 8th iteration of the mower code. We call our code mower because our rocket's **mow the grass** (lawn dart).

featuring airbrakes, mechanical parachute deployment, and a camera system, the smart rocket actively tries to score better in the ARC (American Rocketry Challenge).

## Electronics
We wanted our electronics to have maximium functionality even if it seems  pointless. We choose the latest sensors on the market.

### Microprocessor
#### ESP32 
- boasting `240` mhz and `2` cores, we choose this microprocessor for it's esp32 speed and ease of use.
- The `wifi & bluetooth` components are unused as it triples compile times.
---
### Barometer
#### BMP581 
- `  < 1 meter` precision, `100+` hz.
- It's low latency is good for finding instentaneous velocity through derivative
- A current issue is that it seems to be a bit noisy and data is screwed with by the airbrakes
---

### 9axis IMU, Accelerometer, Gyroscope, and Magnometer
#### BNO085
- `+-8G` acceleration, `100+` hz
- Used to find orientation through sensor fusion
---

### Accelerometer
#### ADXL343 
- `+-16G` acceleration, `100+` hz
- Captures full range of rocket acceleration
- It's filtered **global vertical acceleration** is found through orientation compensation from the BNO085 
---

### SD CARD ADAPTER
- Generic micro sd card reader using SPI.
- Used to log flight data
- we give it 5volts as feeding it 3.3v is not stable (figured out after countless hours of trial and error)
---

### Battery
#### 7.4V 2S 600 mAH LIPO battery
- Very small battery to optimize weight
- ~1 hour battery life
- The electronics have no undervoltage protection so the battery will get destroyed if left on for too long 
- main limiting factor is the number of servos we use. 1S can support 1 servo, 2S can support atleast 5 servos.
---

### Actuation
#### mg90s Servos
- cheap, small, light
- decently powerful. More powerful than the mg90s
- fast actuation
- relatively minimal power consumption
- reliable so far (except for power issues)


## Code
every sensor/peripheral is it's own file for organization. The logic is split into flight states `0 through 5`

### SETUP
- intializes every subsystem, sensors, actuatiors, etc

### State 0
#### DEBUG
- the debug state unused for rocket launches,
- used to print data and other outputs
### State 1
#### LAUNCHPAD READY
- This state corresponds to the rocket on the launch pad awaiting detecting high acceleration or passing an altitude threshold
- Prevents junk data in the data files
- A rolling acceleromter zero is used to have a good altitude and velocity tare after the rocket launch
### State 2
#### THRUST STAGE
- This state corresponds to the rocket motor burn. This prevents the airbrakes from actuating
- In stage 2 & 3, the SD card does not open and close the file written. That's because writing the file halves the frequency of processing loop. We're essentially sacrificing data safety for `2-3x hz`.
### State 3
#### UPWARD COAST
- Standard ballistic trajectory territory.
- Airbrake algorithms are active in this stage to try and coast to the target altitude. *STUB*
- No SD card closing for maximium datarate

### State 4
#### DOWNWARD COAST
- Parachute deployment algorithm activates *STUB*
- opens and closes the data log every loop for safety. Less need for extremely realtime data

### Stage 5
#### On the ground
- no data logging, the rocket remains dossile

## ALGORITHMS *STUB*
### Rolling Accelerometer Tare
### Kalman Filter
### Predict Apogee
### Control Loop
### Parachute Deployment
