#######라즈베리파이#########

import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) 

trig=23
echo=24
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)
GPIO.setup(4,GPIO.OUT)#buzzer

STOP  = 0
FORWARD  = 1
BACKWORD = 2

CH1 = 0
CH2 = 1

OUTPUT = 1
INPUT = 0

HIGH = 1
LOW = 0
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin
#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

MQTT_BROKER = "192.168.0.5"
#"172.18.9.12"
#"172.18.9.12"
MQTT_SEND = "C:\\Users\\bravo\\PycharmProjects\\TINY\\run_webcam_yolov4_DeepSORT"
#"C:\\Users\\bravo\\PycharmProjects\\MQTT_P\\server" #min
#"C:\\Users\\bravo\\PycharmProjects\\TINY\\run_webcam_yolov4_DeepSORT" #seung
client = mqtt.Client()
client.connect(MQTT_BROKER)

cap = cv.VideoCapture(0)

def setPinConfig(EN, INA, INB):
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    pwm = GPIO.PWM(EN, 100)
    pwm.start(0)
    return pwm

def setMotorContorl(pwm, INA, INB, speed, stat):

    pwm.ChangeDutyCycle(speed)
    
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
        
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
        
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)
        

def setMotor(ch, speed, stat):
    if ch == CH1:
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        setMotorContorl(pwmB, IN3, IN4, speed, stat)

def getDistance():
    GPIO.output(trig,False)
    time.sleep(0.07)
    GPIO.output(trig,True)
    time.sleep(0.0001)
    GPIO.output(trig,False)
    while GPIO.input(echo)==0:
        pulse_start=time.time()
    while GPIO.input(echo)==1:
        pulse_end=time.time()
        
    pulse_duration=pulse_end-pulse_start
    distance=round(pulse_duration*17150,0)
    return distance

data = 0

def on_message(client, userdata, message):
    #print("rm: ", str(message.payload.decode("utf-8")))
    global data
    data = str(message.payload.decode("utf-8"))

GPIO.output(4,False)

try:
    GPIO.output(4,True)
    time.sleep(0.5)
    client.loop_start()

    pwmA = setPinConfig(ENA, IN1, IN2)
    pwmB = setPinConfig(ENB, IN3, IN4)
    while True:
        GPIO.output(4,False)
        _, frame = cap.read()
        _, buffer = cv.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer)
        client.publish(MQTT_SEND, jpg_as_text)
        client.subscribe("smart")
        client.on_message = on_message

        distance_value=getDistance()
        #if data == "break": exit()
            
        if data == "center":
            if distance_value > 25:
                print("go", distance_value)
                setMotor(CH1, 40, FORWARD)
                setMotor(CH2, 40, FORWARD)
                time.sleep(0.05)
                #GPIO.output(4,False)
                #print(distance_value)
            elif distance_value >=15 and distance_value<=25:
                setMotor(CH1, 80, STOP)
                setMotor(CH2, 80, STOP)
                time.sleep(0.05)
                #GPIO.output(4,False)
            elif distance_value < 15:
                setMotor(CH1, 40, BACKWORD)
                setMotor(CH2, 40, BACKWORD)
                time.sleep(0.05)
                #GPIO.output(4,False)

        elif data == "right":
            if distance_value >15 and distance_value<=25:
                setMotor(CH1, 80, STOP)
                setMotor(CH2, 80, STOP)
                time.sleep(0.05)
                #GPIO.output(4,False)
            elif distance_value < 15:
                setMotor(CH1, 40, BACKWORD)
                setMotor(CH2, 40, BACKWORD)
                time.sleep(0.05)
                #GPIO.output(4,False)
            #print(data)
            else:
                setMotor(CH1, 0, BACKWORD) #
                setMotor(CH2, 70, FORWARD)#power up
                time.sleep(0.05)
        elif data == "m_right":
            if distance_value >=15 and distance_value<=25:
                setMotor(CH1, 80, STOP)
                setMotor(CH2, 80, STOP)
                time.sleep(0.05)
                #GPIO.output(4,False)
            elif distance_value < 15:
                setMotor(CH1, 40, BACKWORD)
                setMotor(CH2, 40, BACKWORD)
                time.sleep(0.05)
                #GPIO.output(4,False)
            #print(data)
            else:
                setMotor(CH1, 0, BACKWORD) #
                setMotor(CH2, 55, FORWARD)#power up
                time.sleep(0.05)
                #print(distance_value)
            #GPIO.output(4,False)
        elif data == "left":
            if distance_value >=15 and distance_value<=25:
                setMotor(CH1, 80, STOP)
                setMotor(CH2, 80, STOP)
                time.sleep(0.05)
                #GPIO.output(4,False)
            elif distance_value < 15:
                setMotor(CH1, 40, BACKWORD)
                setMotor(CH2, 40, BACKWORD)
                time.sleep(0.05)
                #GPIO.output(4,False)
            else:
                setMotor(CH1, 70, FORWARD)#power up
                setMotor(CH2, 0, BACKWORD)
                time.sleep(0.05)
        elif data == "m_left":
            if distance_value >=15 and distance_value<=25:
                setMotor(CH1, 80, STOP)
                setMotor(CH2, 80, STOP)
                time.sleep(0.05)
                #GPIO.output(4,False)
            elif distance_value < 15:
                setMotor(CH1, 40, BACKWORD)
                setMotor(CH2, 40, BACKWORD)
                time.sleep(0.05)
                #GPIO.output(4,False)
            else:
                setMotor(CH1, 55, FORWARD)#power up
                setMotor(CH2, 0, BACKWORD)
                time.sleep(0.05)
                #print(distance_value)
            #GPIO.output(4,False)
        elif data == "not_track":
            setMotor(CH1, 80, STOP)
            setMotor(CH2, 80, STOP)
            #GPIO.output(4,True)
            #time.sleep(0.05)

    setMotor(CH1, 80, STOP)
    setMotor(CH2, 80, STOP)
    #GPIO.output(4,False)

except:
    cap.release()
    client.loop_end()
    GPIO.cleanup()
    #client.disconnect()

    print("\nNow you can restart fresh")
