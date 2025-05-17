# ESP32-C3-0.42-OLED-Pendant
I made a little pendant from one of those aliexpress 0.42' OLED ESP32-C3 boards, an SHTC3 and SGP40 sensor, and an INMP441 I2s mic.
![OLEDPendantWHITEBG](https://github.com/user-attachments/assets/9c91e150-adc6-4baa-be38-daf8d6107dad)


## Wiring:
  ### SHTC3/SGP40:    ESP32-C3 GPIO  
    Vin    --->    3V pin
    GND    --->    GND
    SCL    --->    (6)(SCL)   (Any online picture is wrong, these are the correct pins to use for i2c communication, and are the pins the display is hardwired to. )
    SDA    --->    (5)(SDA)  ^

 ### INMP441:
    VDD    --->    3V
    GND    --->    GND
    WS     --->    (7)
    SCK    --->    (10)
    SD     --->    (2)
    L/R    --->    GND (This sets the mic as the left input. if using two modules, connect the second modules (L/R) to 3V instead, or a free pin that you pull up in your setup code. I didn't bother with all that, so I just connected the (L/R) pin to the grounded edge of the module.)

 ### OLED (hardwired, cannot be changed):
    SDA   --->    (5)
    SCL   --->    (6)


## Note:
  You can find this image online, and similar ones with the same pinout. none of them list 5 and 6 as SCL and SDA, and instead label pins 8 and 9. 8 and 9 can work as an i2c address, but causes issues when trying to use the display at the same time. The solution is to use pins 5 and 6, and make sure to properly handle the i2c devices taking turns in your code. ![esp32c3superminioledpinoout](https://github.com/user-attachments/assets/ee8b3423-a1df-4d5a-a656-fcc2e5074103)

### Here's a more accurate version to reference:
I didn't bother with showing the SPI pins in this one, but it's fine for i2c device projects.

![esp32c3superminioledpinooutcorrected](https://github.com/user-attachments/assets/275c3bca-3713-4870-8ae1-c3ba95ff9e08)



  
  
