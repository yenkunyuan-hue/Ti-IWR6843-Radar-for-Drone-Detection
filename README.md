# Aalborg Univeristy, AIE-2 project
THANK YOU to Christian Mai and Fredrik F. Sørensen, for their guidance throughout this project.
<br>THANK YOU to [Yash](https://github.com/yashlancers/mm_Wave_Radar_IWR6843AOPEVM), his IWR code is the initial version of radar.py
<br>THANK YOU to [Alireza0K](https://github.com/Alireza0K/Unmanned-Aerial-Vehicle) for his open-sourced yolo-v8 detection model
<br>Author: Yang Kunyuan(me), Maté Papp, Douglas Takle

## Python Environment: 
Interpreter: Python 3.8
<br>ultralytics==8.4.37
<br>pyserial==3.5
<br>scikit-learn==1.3.2
<br>yolo model's [weighted file](https://github.com/yenkunyuan-hue/Ti-IWR6843-Radar-for-Drone-Detection/blob/main/Model.pt)
<br>radar's [configuration file](https://github.com/yenkunyuan-hue/Ti-IWR6843-Radar-for-Drone-Detection/blob/main/config_file.cfg), come from [website](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/)
<br>SDK==3.6
<br>platform==xWR68xx_AOP

## Keyword:
Ti IWR6843AOPEVM
<br>Yolov8 Model Implementation
<br>Arduino's Junior Project
<br>Drone Detection

## Project Introduction:
Working Video:
<br>https://github.com/yenkunyuan-hue/Ti-IWR6843-Radar-for-Drone-Detection/raw/main/example.mp4
<br>This prototype is used for UAV detection, clour in LED means:
<br>Green - Detected
<br>Red - Python is crashed
<br>Blue - No Detection

## Target User: 
bachelor or high school student who is interested in AI's implementation and Arduino
<br>WE PROVIDE A DETAILED EXPLAINATION FOR EACH COMPONENT:
<br>https://www.overleaf.com/read/yjyrxjvpsxby#cf75ce
<br>Chapter 4,5,6 are STRONGLY RECOMMENDED to read if you are the beginner

## Device List: 
Ti IWR6843AOPEVM - can change to other radar, need to edit radar.py if the format is insame 
<br>HC-SR04 Ultrasonic Sensor
<br>Arduino Uno R3
<br>FIT0458 Motor with Encoder - encoder will not be used
<br>L298N Motor Driver - NECESSARY if you don't want to burn your Arduino
<br>Common Anoder RGB LED - Cathode is okay too, but you need to change AnalogWrite in LED_show() in Arduino.ino
<br>Light Dependent Resistor
<br>Jetson Nano - Raspaberry Pi is okay too

## Testing Performance: 
NOTE: Distance here is Horizontal Distance, Vertical Distacne is varying
<br>![set_1](./33cm-test.png)
<br>![set_2](./175cm-test.png)
<br>![set_3](./290cm-test.png)
<br>![set_4](./330cm-test.png)

## 3D Printing:
Two format of 3D printing files are provided;
[STL](https://github.com/yenkunyuan-hue/Ti-IWR6843-Radar-for-Drone-Detection/blob/main/Radar.stl) and [f3d](https://github.com/yenkunyuan-hue/Ti-IWR6843-Radar-for-Drone-Detection/blob/main/Radar.f3d)

## Possible Improvement:
KNN is a better algorithm than OPTICS in processing the point cloud
However, KNN is a supervised learning - you need to collect a training set for a specific device

## If Question:
feel free to contact me, Yang, (fy14pi@student.aau.dk)
