
# Rear-End Collision Avoidance Project (RECAP)

Repository for storing code relating to MTE 481/482 Capstone Design Project

## Environment Set-up

### Arduino IDE
Follow the guides provided by Adafruit to add the board support packages to the arduino IDE.
* [Step 1: Add the Adafruit repository to the package manager](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/setup)
* [Step 2: Install the board support packages](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-with-arduino-ide)

### Libraries
Download and install the following libraries under `C:\Users\username\Documents\Arduino\libraries`

Scheduler:

Use the Library Manager to install the **Scheduler** library.

Adafruit TFT Library:

Use the Library Manager to install the **Adafruit GFX Library** and **Adafruit HX8357 Library**.

Time Library:

```
git clone https://github.com/PaulStoffregen/Time.git
```

Adafruit GPS Library:

```
git clone https://github.com/adafruit/Adafruit_GPS.git
```

Modified LoRa Library:

```
git clone https://github.com/rcswift/arduino-LoRa.git
```

Modified Freematics OBD2 Library:

```
git clone https://github.com/rcswift/ArduinoOBD.git
```

Important Note: Must include the contents of the library folder not the git repo.

## Useful Links:

* [Freematics Adapter](https://freematics.com/products/freematics-obd-ii-uart-adapter-mk2/)
* [Adafruit Feather M0 RFM95W LoRa Product Guide](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module)
* [Adafruit Ultimate GPS FeatherWing Product Guide](https://learn.adafruit.com/adafruit-ultimate-gps-featherwing)
* [Adafruit 3.5" TFT FeatherWing Product Guide](https://learn.adafruit.com/adafruit-3-5-tft-featherwing)
