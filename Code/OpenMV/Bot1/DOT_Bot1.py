# Untitled - By: boris - Сб ноя 16 2019
#-------------------------------------------------
#CX = 149; CY = 120
ball_0cm = 0
ball_5cm = 7
ball_10cm = 13
ball_15cm = 20
ball_20cm = 28
ball_25cm = 34
ball_30cm = 40
ball_35cm = 45
ball_40cm = 51
ball_45cm = 56
ball_50cm = 60
ball_55cm = 65
ball_60cm = 68
ball_65cm = 72
ball_70cm = 75
ball_75cm = 78
ball_80cm = 81
ball_85cm = 85
ball_90cm = 87
ball_95cm = 89
ball_100cm = 91
ball_105cm = 93
ball_110cm = 95
ball_115cm = 96
ball_120cm = 97
ball_125cm = 97
ball_130cm = 98
ball_135cm = 99
ball_140cm = 100
ball_145cm = 101
ball_150cm = 102
#---------------------------------------------------

#---------------------------------------------------
def realDistance(x):
    fSign = 1
    if x<0:
        xAbs = x*-1
        fSign = -1
    else:
        xAbs = x
    if(ball_0cm <= xAbs and xAbs < ball_5cm):
        xOne = ball_0cm
        xTwo = ball_5cm
        yOne = 0
        yTwo = 5
    elif(ball_5cm <= xAbs and xAbs < ball_10cm):
        xOne = ball_5cm
        xTwo = ball_10cm
        yOne = 5
        yTwo = 10
    elif(ball_10cm <= xAbs and xAbs < ball_15cm):
        xOne = ball_10cm
        xTwo = ball_15cm
        yOne = 10
        yTwo = 15
    elif(ball_15cm <= xAbs and xAbs < ball_20cm):
        xOne = ball_15cm
        xTwo = ball_20cm
        yOne = 15
        yTwo = 20
    elif(ball_20cm <= xAbs and xAbs < ball_25cm):
        xOne = ball_20cm
        xTwo = ball_25cm
        yOne = 20
        yTwo = 25
    elif(ball_25cm <= xAbs and xAbs < ball_30cm):
        xOne = ball_25cm
        xTwo = ball_30cm
        yOne = 25
        yTwo = 30
    elif(ball_30cm <= xAbs and xAbs < ball_35cm):
        xOne = ball_30cm
        xTwo = ball_35cm
        yOne = 30
        yTwo = 35
    elif(ball_35cm <= xAbs and xAbs < ball_40cm):
        xOne = ball_35cm
        xTwo = ball_40cm
        yOne = 35
        yTwo = 40
    elif(ball_40cm <= xAbs and xAbs < ball_45cm):
        xOne = ball_40cm
        xTwo = ball_45cm
        yOne = 40
        yTwo = 45
    elif(ball_45cm <= xAbs and xAbs < ball_50cm):
        xOne = ball_45cm
        xTwo = ball_50cm
        yOne = 45
        yTwo = 50
    elif(ball_50cm <= xAbs and xAbs < ball_55cm):
        xOne = ball_50cm
        xTwo = ball_55cm
        yOne = 50
        yTwo = 55
    elif(ball_55cm <= xAbs and xAbs < ball_60cm):
        xOne = ball_55cm
        xTwo = ball_60cm
        yOne = 55
        yTwo = 60
    elif(ball_60cm <= xAbs and xAbs < ball_65cm):
        xOne = ball_60cm
        xTwo = ball_65cm
        yOne = 60
        yTwo = 65
        if(ball_65cm <= xAbs and xAbs < ball_70cm):
        xOne = ball_65cm
        xTwo = ball_70cm
        yOne = 65
        yTwo = 70
    elif(ball_70cm <= xAbs and xAbs < ball_75cm):
        xOne = ball_70cm
        xTwo = ball_75cm
        yOne = 70
        yTwo = 75
    elif(ball_75cm <= xAbs and xAbs < ball_80cm):
        xOne = ball_75cm
        xTwo = ball_80cm
        yOne = 75
        yTwo = 80
    elif(ball_80cm <= xAbs and xAbs < ball_85cm):
        xOne = ball_80cm
        xTwo = ball_85cm
        yOne = 80
        yTwo = 85
    elif(ball_85cm <= xAbs and xAbs < ball_90cm):
        xOne = ball_85cm
        xTwo = ball_90cm
        yOne = 85
        yTwo = 90
    elif(ball_90cm <= xAbs and xAbs < ball_95cm):
        xOne = ball_90cm
        xTwo = ball_95cm
        yOne = 90
        yTwo = 95
    elif(ball_95cm <= xAbs and xAbs < ball_100cm):
        xOne = ball_95cm
        xTwo = ball_100cm
        yOne = 95
        yTwo = 100
    elif(ball_100cm <= xAbs and xAbs < ball_105cm):
        xOne = ball_100cm
        xTwo = ball_105cm
        yOne = 100
        yTwo = 105
    elif(ball_105cm <= xAbs and xAbs < ball_110cm):
        xOne = ball_105cm
        xTwo = ball_110cm
        yOne = 105
        yTwo = 110
    elif(ball_110cm <= xAbs and xAbs < ball_115cm):
        xOne = ball_110cm
        xTwo = ball_115cm
        yOne = 110
        yTwo = 115
    elif(ball_115cm <= xAbs and xAbs < ball_120cm):
        xOne = ball_115cm
        xTwo = ball_120cm
        yOne = 115
        yTwo = 120
    elif(ball_120cm <= xAbs and xAbs < ball_125cm):
        xOne = ball_120cm
        xTwo = ball_125cm
        yOne = 120
        yTwo = 125
    elif(ball_125cm <= xAbs and xAbs < ball_130cm):
        xOne = ball_125cm
        xTwo = ball_130cm
        yOne = 125
        yTwo = 130
    elif(ball_130cm <= xAbs and xAbs < ball_135cm):
        xOne = ball_130cm
        xTwo = ball_135cm
        yOne = 130
        yTwo = 135
    elif(ball_135cm <= xAbs and xAbs < ball_140cm):
        xOne = ball_135cm
        xTwo = ball_140cm
        yOne = 135
        yTwo = 140
    return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign








#---------------------------------------------------

import sensor, image, time, pyb
from math import sqrt, atan2
from pyb import UART
from pyb import SPI
EXPOSURE_TIME_SCALE = 0.4

#txrx = UART(3, 115200, timeout_char = 1)
#txrx.init(115200, bits=8, parity=None, stop=1,timeout_char = 1)
#spi = SPI(2, SPI.SLAVE, polarity=0, phase=0, crc = 0x08)

buf = bytearray(9)


cX = 149  # bot 111111111111111111111111111111111111111111111111111111111111111111111111
cY = 120  # bot 111111111111111111111111111111111111111111111111111111111111111111111111


ballX = 0
ballY = 0
yGoalX = 0
yGoalY = 0
bGoalX = 0
bGoalY = 0

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time = 500)

sensor.set_auto_whitebal(True)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False)
current_exposure_time_in_microseconds=  sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds* EXPOSURE_TIME_SCALE))
sensor.skip_frames(time = 10000)

clock = time.clock()


def crc8(data, len):
    crc = 0xFF
    j = 0
    for i in range(0, len):
        crc = crc ^ data[i];
        for j in range(0, 8):
            if (crc & 0x80):
                crc = (crc << 1) ^ 0x31
            else:
             crc = crc << 1
    return crc


threshold_yellow=(32, 100, -21, 30, 34, 127)
threshold_ball=(18, 84, 40, 127, 4, 125)
#pyb.ExtInt(pyb.Pin("P3"), pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, nss_callback)
while(True):
    clock.tick()
    img = sensor.snapshot()


#---Yellow-Goal-Values-------------------------
    centerYYellow = -1
    centerXYellow = -1
    GLYYellow = -1
    GLXYellow = -1
    angleYellow = -1
    lineYellow = -1
    sideYellow = -1
    distanceYellow = -1
#--------------------------------------------

#---Yellow-Goal-Finder-------------------------
    blobsPreviousMas = 0
    biggestYellowBlob = 0
    count =0
    blobsYellow=[]
    for yellowBlob in img.find_blobs([threshold_yellow],roi=(0,0,319,239), pixels_threshold=10, area_threshold=10, merge=True):
        blobsYellow.append(yellowBlob)
        if(yellowBlob.area() > blobsPreviousMas):
            biggestYellowBlob = count
            blobsPreviosMas = yellowBlob.area()
        count+=1
    try:
        img.draw_rectangle(blobsYellow[biggestYellowBlob].rect())
        img.draw_cross(blobsYellow[biggestYellowBlob].cx(),blobsYellow[biggestYellowBlob].cy(), 6)
    except: pass
    maxLeft = 320
    maxRight = 0
    count = 0
    indL = 0
    indR = 0
    centerY = 0
    centerX = 0

    try:
        centerY = blobsYellow[biggestYellowBlob].cy()
        centerX = blobsYellow[biggestYellowBlob].cx()
        GLXPixel = -centerX + cX
        GLYPixel = -centerY + cY
        #print(GLYPixel)
        angle = atan2(GLXPixel, GLYPixel)
        angle = (angle*57)//1
        angleYellow = angle
        #print((GLXPixel + 139) // 2)
        yGoalX = (GLXPixel + 139) // 2
        yGoalY = (GLYPixel + 139) // 2
    except: pass
#----------------------------------------------

#----------Ball-Values-------------------------
    centerYBall = -1
    centerXBall = -1
    GLYBall = -1
    GLXBall = -1
    angleBall = -1
    lineBall = -1
    sideBall = -1
    distanceBall = -1
#--------------------------------------------

    blobsPreviousMas = 0
    biggestBallBlob = 0
    count =0
    blobsBall = []
    for ballBlob in img.find_blobs([threshold_ball],roi=(0,0,319,239), pixels_threshold=10, area_threshold=10, merge=True):
        blobsBall.append(ballBlob)
        if(ballBlob.area() > blobsPreviousMas):
            biggestballBlob = count
            blobsPreviosMas = ballBlob.area()
        count+=1
    try:
        img.draw_rectangle(blobsBall[biggestBallBlob].rect())
        img.draw_cross(blobsBall[biggestBallBlob].cx(),blobsBall[biggestBallBlob].cy(), 6)
    except: pass
    maxLeft = 320
    maxRight = 0
    count = 0
    indL = 0
    indR = 0
    centerY = 0
    centerX = 0

    try:
        centerY = blobsBall[biggestBallBlob].cy()
        centerX = blobsBall[biggestBallBlob].cx()
        GLXPixel = -centerX + cX
        GLYPixel = -centerY + cY
        #print(GLXPixel)
        angle = atan2(GLXPixel, GLYPixel)
        angle = (angle*57)//1
        angleBall = angle
        #print((GLXPixel + 139) // 2)
        ballX = (GLXPixel + 139) // 2
        ballY = (GLYPixel + 139) // 2

        print("GLXPixel: " + str(GLXPixel) + " GLYPixel: " + str(GLYPixel))

    except: pass
    spi = SPI(2, SPI.SLAVE, polarity=0, phase=0)
    #print("kek")
    buf[0] = 0xBB
    buf[1] = 6 #msg_length
    buf[2] = ballX
    buf[3] = ballY
    buf[4] = yGoalX
    buf[5] = yGoalY
    buf[6] = 1
    buf[7] = 1
    buf[8] = crc8(buf, 8)

    spi.write(buf)

    spi.deinit()
    img.draw_cross(cX, cY, (0,255,0))
   # print(clock.fps())
