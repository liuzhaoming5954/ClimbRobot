# ClimbRobot
Climbing Robot DPS code
We use the chip ESP8266 to connect the DSP with WIFI [ESP8266](https://en.wikipedia.org/wiki/ESP8266)    
When you have a new ESP8266, you should configure it at first with **AT commands**.  [AT commands set](https://www.espressif.com/sites/default/files/documentation/4a-esp8266_at_instruction_set_en.pdf)     
Firstly, you need a serial port debugging tool to configure, like this [one](https://github.com/Neutree/COMTool/releases).


**These code for ESP8266**	
```
   //SCIC for debug ESP chip by computer
   // Connect to the Router
   msg = "AT+CWJAP=\"Robotics Lab 518a\",\"ccny10031\"\r\n\0"; // if you change a router, then change the name and password
   scic_msg(msg); // connect scic with computer, you can see the feedback from ESP on computer
   scib_msg(msg);
   DELAY_US(5000000); //等待微妙 wait for some microseconds

   //Set by TCP multiple connections
   msg = "AT+CIPMUX=1\r\n\0";
   scic_msg(msg);
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   //Set ESP as server, port=8888
   msg = "AT+CIPSERVER=1,8888\r\n\0";
   scic_msg(msg);
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   // Get IP Address, if you connect with computer and open scic, you will see the IP address on computer
   msg = "AT+CIFSR\r\n\0";
   scic_msg(msg);
   scib_msg(msg);
```
