# DuvelBot - An ESP32-CAM beer server robot

![DuvelBot](https://github.com/larsupilami73/DuvelBot/blob/master/schematic/P1000412.JPG)

Duvelbot is a simple ESP32-CAM based web-controlled robot, programmable from the Arduino IDE.
It shows how to combine HTML (buttons/sliders), CSS, async server requests, motor and leds control into
a browser-controlled driving camera (that can bring your beer). 
The interface looks like this:

![webinterface](https://github.com/larsupilami73/DuvelBot/blob/master/schematic/webinterface1.jpg)

## How to program:

* Install ESP32 libraries, SPIFFS, Async webserver library in the Arduino IDE 
  as explained at [Randomnerdtutorials](https://randomnerdtutorials.com/esp32-web-server-spiffs-spi-flash-file-system/).
* Change your network credentials in the sketch:
  ```
  const char* ssid = "yourNetworkSSIDHere";
  const char* password = "yourPasswordHere";
  ```
* Upload the sketch data and sketch itself using a USB-to-3.3V-TTL converter
* Open a browser and go to the webaddress it reported on the serial monitor of the Arduino IDE during boot.

## Schematic:

![DuvelBot](https://github.com/larsupilami73/DuvelBot/blob/master/schematic/DuvelBot_schematic.png)

## Todos/extentions/ideas:

* Move camera streaming to a 2nd server like [here](https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/) on the same ESP32 and import the video into the html served by the first server using `<iframe>...</iframe>`.
* Use access point (AP) mode so the robot is more standalone as explained [here](https://randomnerdtutorials.com/esp32-access-point-ap-web-server/).
* Expand with battery voltage measurement, deep-sleep capabilities etc. (difficult because the ESP32-CAM doesn't have many GPIOs).

