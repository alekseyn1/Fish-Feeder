# Fish-Feeder (non-blocking)
ESP8622-based automatic fish feeder with Home Assistant Auto Discovery of the Fish Feeder Switch and Fish Light Switch

The system is built based on the Wemos D1 mini
Fish Feeder is based on https://www.thingiverse.com/thing:3729421

D1 Mini also controls the relay that turns on/off the light. Timing of the lights on/off controlled via Home Assistant Automation
Did not have time to complete the relay on/off via the firmware

Feeding is in the firmware but can also be achieved via Home Assistant if required (I chose to do it all via ESP8622 firmware) by commenting out line 345 [feeding();]
The feeding switch will turn off as soon as the movement of the feeder completes (1-2 sec)
The amount of food is controlled by the width of the opening of the traveling piece inside the feeder.
Included design seems to be the correct size for a single betta fish with feed times at 8:00 and 20:00
