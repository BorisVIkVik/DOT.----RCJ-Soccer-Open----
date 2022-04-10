# Untitled - By: boris - Сб ноя 16 2019

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
sensor.skip_frames(time = 2000)

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


threshold_yellow=(38, 100, 3, 45, 30, 127)
threshold_ball=(51, 71, 56, 113, 28, 58)
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
        print(GLYPixel)
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
