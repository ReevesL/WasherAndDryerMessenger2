# WasherAndDryerMessenger2
A rewrite of WasherAndDryerMessenger targeting Arduino Mega with an Adafruit CC3000

I've started the process of rewriting the app to switch to using an Adafruit CC3000 wifi board. This board was an early, inexpensive wifi board from Adafruit using a chip by TI. This was the least expensive board at the time I bought it, but it has a number of limitations, most critically a lack of support for HTTPS.

Between the lack of secure connections, larger libraries and a desire to move to use the IFTTT.com maker channel I opted to just use an Arduino Mega instead and many memory constraints were removed (as well as pin count limitations).
