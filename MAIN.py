import atexit
from ultralytics import YOLO
import cv2 as cv
import os
import csv
import serial
from radar import serialConfig,update,parseConfigFile
import time
import numpy as np
from sklearn.cluster import OPTICS

serial_port = ('/dev/cu.usbmodem1301')
ser = serial.Serial(serial_port, 9600, timeout=1)
image = 0
output_img_all = 'Images'
output_img_dir = 'Detected Result'
output_acc_dir = 'Confidence.csv'

cap1 = cv.VideoCapture(0)
cap2 = cv.VideoCapture(1)
configFileName = 'config_file.cfg'
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2 ** 15, dtype='uint8')
byteBufferLength = 0

if not os.path.exists(output_img_dir):
    os.makedirs(output_img_dir)
if not os.path.exists(output_img_all):
    os.makedirs(output_img_all)
if not os.path.exists(output_acc_dir):
    with open(output_acc_dir, 'w', newline='') as csvfile:
        file = csv.writer(csvfile)
        file.writerow(['Image Name', 'Accuracy', 'Moving Cloud Points', 'Number of Object', 'Distance', 'Light' ])

# Load the custom YOLOv8 model

model = YOLO('Model.pt')
def exist():
    ser.write("e".encode('utf-8'))

def prediction(img):
    accuracy = []
    save_path = os.path.join(output_img_dir, name+'.jpg')
    #img = cv.imread(img) #it is necessary if we load picture directly into it
    results = model(img, save = False, stream = True, verbose=False)
    for r in results:
        if len(r.boxes) == 0:
            print('no detection')
            accuracy = [0]
        else:
            print(r.boxes)
            accuracy = r.boxes.conf.tolist()
            print('there is a drone')
            annotated_frame = r.plot()
            cv.imwrite(save_path, annotated_frame)
    with open(output_acc_dir, 'a', newline='') as csvfile:
        file = csv.writer(csvfile)
        for i in range(0,len(accuracy)):
            write_temp = write
            write_temp[0] = name
            write_temp[1] = accuracy[i]
            file.writerow(write_temp)
            if accuracy[i] > 0:
                ser.write("r\n".encode('utf-8'))

def imag_1_capture():
    global image
    save_all_path = os.path.join(output_img_all, name + '.jpg')
    cap1.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap1.set(cv.CAP_PROP_FRAME_HEIGHT, 640)
    ret, frame = cap1.read()
    if ret is True:
        cv.imwrite(save_all_path, frame)
        image = frame
        print('camera 1 picture taken')
    else:
        print('Camera 1 Fault')


def imag_2_capture():
    global image
    save_all_path = os.path.join(output_img_all, name + '.jpg')
    cap2.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv.CAP_PROP_FRAME_HEIGHT, 640)
    ret, frame = cap2.read()
    if ret is True:
        cv.imwrite(save_all_path, frame)
        image = frame
        print('camera 2 picture taken')
    else:
        print('Camera 2 Fault')


def MCU_read():
    data = ''
    check = False
    ser.reset_input_buffer()
    while data == '' or check == False:
        data = ser.readline()
        data = data.decode('utf-8').strip()
        data = data.split('S')
        if len(data)==3:
            data = data[1].split(";")
            print(data)
            if len(data)>=3:
                check = True
    Status = data[0]
    Distance = data[1]
    Light = data[2]
    print(data)
    return Status, Distance, Light

# Main loop
detObj = {}
frameData = {}
currentIndex = 0
number_detect = 0
num_moving_point = 0
num_cluster = 0
atexit.register(exist)
while True:
    write = [0, 0, 0, 0, 0, 0] #'Image Name', 'Accuracy', 'Moving Cloud Points', 'Number of Object', 'Distance', 'Light'
    number_detection = [0, 0, 0]     #number of moving points, number of object, number of loop
    name = time.strftime('%H%M%S')
    acoustic_stat, write[4], write[5] = MCU_read()
    if acoustic_stat == 'False':
        write[4] = 'NA'
    while number_detection[2] <=10:
        # Update the data and check if the data is okay
        try:
            data_status, num_cloud_point, num_moving_point, num_cluster = update()
            if data_status:
            # Store the current frame into frameData
                number_detection[0] += num_moving_point
                if (num_cluster == 0 and num_moving_point>0):
                    number_detection[1] += 1
                else:
                    number_detection[1] += num_cluster
                number_detection[2] +=1
        except KeyboardInterrupt:
            CLIport.write(('sensorStop\n').encode())
            CLIport.close()
            Dataport.close()
            break
        # Sampling frequency of 30 Hs
    number_detection[0] = round(number_detection[0]/number_detection[2])
    number_detection[1] = round(number_detection[1]/number_detection[2])
    print(f'number of moving point in average{number_detection[0]}')
    print(f'number of object in average{number_detection[1]}')
    if number_detection[0] >= 1 or number_detection[1] >=1:
        print('camera start to work')
        write[3] = number_detection[0]
        write[2] = number_detection[1]
        if int(write[5])>=200:
            imag_1_capture()
            print('camera 1 finished work')
        else:
            imag_2_capture()
            print('camera 2 finished work')
        prediction(image)
    else:
        print('no camera works')
    print('1 loop finished')