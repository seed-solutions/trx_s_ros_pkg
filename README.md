# trx_s_ros_pkg

## Pre-requirement

### About TRX-S
[SEED Solutions HP](http://seed-solutions.net/)  
[Manuals, etc.](http://seed-solutions.net/?q=node/7)

### Basic Configuration  
![basicconfiguration](https://user-images.githubusercontent.com/12426780/42069297-ec0f0c66-7b8b-11e8-9703-9b48371c76ce.jpg)

## How to use

### Basic usage
bringup the controller node and Rviz  
```
roslaunch trx_s_ros_pkg bringup.launch
```  
grasp  
```
rosservice call /hand_controller grasp
```  
open  
```
rosservice call /hand_controller open
```  

### Parameters (written in the bringup.launch)
#### serial_port (default = "ttyACM0")  
It depends on the your communication module.  
In case of CM4U (TRX-S Academic Package), it's ttyACM*  
In case of CMSU, it's ttyUSB*  

#### can_id (default = 1)  
If you change the CAN ID by the SEED-Editor, you should change this parameter to same one.

#### controller_rate(default = 20 )  
This parameter is the interval of getting current position.

### Other
* Add user to dialout  
```sudo gpasswd -a <user-name> dialout```
