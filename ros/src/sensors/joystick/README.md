# Welcome to Joy

The joystick package for the self-driving vehicle project. 

Publishes: `/sensor/joystick/joy`

Message Type: `joy`

Publishes: `/sensor/joystick/enabled`

Message Type: `Bool`
 
The purpose of this is to switch between manual input and autonomous input. If joystick is disabled, the
vehicle automatically listens to autonomous inputs. 

min 190
max 680
