# Untitled - By: boris - Сб ноя 16 2019
#-------------------------------------------------
#CX = 149; CY = 120 dobav k nazvaniyam peremenuh radius robota
ball_0cm = 0
ball_15cm = 15
ball_20cm = 21
ball_25cm = 28
ball_30cm = 35
ball_35cm = 42
ball_40cm = 48
ball_45cm = 53
ball_50cm = 58
ball_55cm = 62
ball_60cm = 68
ball_65cm = 70
ball_70cm = 75
ball_75cm = 79
ball_80cm = 82
ball_85cm = 87
ball_90cm = 90
ball_95cm = 92
ball_100cm = 95
ball_105cm = 98
ball_110cm = 99
ball_115cm = 100
ball_120cm = 102
ball_125cm = 104
ball_130cm = 105

#ballDone
ballCoords = [[0, 0],
              [15,31],
              [20,39],
              [25, 48],
              [30, 54],
              [35, 60],
              [40, 64],
              [45, 67],
              [50, 71],
              [55, 73],
              [60, 75],
              [65, 77],
              [70, 78],
              [75, 81],
              [80, 81],
              [85, 83],
              [90, 85],
              [95, 86],
              [100, 87],
              [105, 88],
              [110, 89],
              [115, 90],
              [120, 91],
              [125, 92]]
#---------------------------------------------------
#done
goalCoords = [[0, 0],
              [9, 18],
              [20, 45],
              [30, 59],
              [40, 66],
              [50, 76],
              [60, 81],
              [70, 87],
              [80, 90],
              [90, 93],
              [100, 95],
              [110, 97],
              [120, 98],
              [130, 100],
              [140, 102],
              [150, 103],
              [160, 104],
              [170, 105],
              [180, 106]]


#---------------------------------------------------
def realDistance(x, coordsArr):
    xOne = 0
    xTwo = 0
    yOne = 0
    yTwo = 0
    fSign = 1
    if x<0:
        xAbs = x*-1
        fSign = -1
    else:
        xAbs = x
    for i in range(0, len(coordsArr) - 1):
        if coordsArr[i][1] <= xAbs and xAbs < coordsArr[i + 1][1]:
            xOne = coordsArr[i][1]
            xTwo = coordsArr[i + 1][1]
            yOne = coordsArr[i][0]
            yTwo = coordsArr[i + 1][0]
            return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign
    if coordsArr[len(coordsArr) - 1][1] <= xAbs:
        xOne = coordsArr[len(coordsArr) - 2][1]
        xTwo = coordsArr[len(coordsArr) - 1][1]
        yOne = coordsArr[len(coordsArr) - 2][0]
        yTwo = coordsArr[len(coordsArr) - 1][0]
    return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign









#---------------------------------------------------

import sensor, image, time, pyb
from math import sqrt, atan2, cos, sin
from pyb import SPI
EXPOSURE_TIME_SCALE = 0.8



threshold_blue =    (12, 86, -98, 117, -92, -27)#(47, 100, -30, 48, 42, 127) yellow
threshold_yellow =  (20, 80, -27, 31, 21, 127)#(1, 9, -1, 11, -27, -7)#(2, 23, 5, 72, -78, -19)#(12, 100, 14, 127, -104, -33) blue
threshold_ball =    (20, 80, 30, 127, 7, 105)#(55, 100, 53, 127, -9, 127)#(50, 65, 49, 127, 23, 127)


cX = 164  # bot 111111111111111111111111111111111111111111111111111111111111111111111111
cY = 123  # bot 111111111111111111111111111111111111111111111111111111111111111111111111

buf = bytearray(10)
ballX = 0
ballY = 0
yGoalX = 0
yGoalY = 0
bGoalX = 0
bGoalY = 0

spi = SPI(2, SPI.SLAVE, polarity=0, phase=0)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
#sensor.set_auto_gain(True)
#sensor.set_auto_exposure(True)
#sensor.set_auto_whitebal(True)
sensor.skip_frames(time = 1000)

#sensor.set_auto_whitebal(True)
#sensor.set_auto_gain(False)
sensor.set_auto_exposure(False)
current_exposure_time_in_microseconds=  sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds* EXPOSURE_TIME_SCALE))
sensor.skip_frames(time = 500)

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


while(True):
    clock.tick()
    img = sensor.snapshot()#.gamma_corr(gamma = 0.3, contrast = 10.0, brightness = 0.0)#.negate()


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
    biggestYellowBlob = -1
    countYellow =0
    blobsYellow=[]
    for yellowBlob in img.find_blobs([threshold_yellow],roi=(0,0,319,239), pixels_threshold=150, area_threshold=150, merge=True, margin = 15):
        blobsYellow.append(yellowBlob)
        if(yellowBlob.area() > blobsPreviousMas):
            biggestYellowBlob = countYellow
            blobsPreviosMas = yellowBlob.area()
        countYellow+=1
    try:
        #img.draw_rectangle(blobsYellow[biggestYellowBlob].rect())
        img.draw_cross(blobsYellow[biggestYellowBlob].cx(),blobsYellow[biggestYellowBlob].cy(), 6)
    except: pass
    maxLeft = 320
    maxRight = 0
    count = 0
    indL = 0
    indR = 0
    centerY = 0
    centerX = 0

    for yel, i in enumerate(blobsYellow):
        if (maxLeft > blobsYellow[yel].x()):
            maxLeft = blobsYellow[yel].x()
            indL = count
        if (maxRight < blobsYellow[yel].cx() + blobsYellow[yel].cx() - blobsYellow[yel].x()):
            maxRight = blobsYellow[yel].cx() + blobsYellow[yel].cx() - blobsYellow[yel].x()
            indR = count
        count+=1

    try:
        centerY = (blobsYellow[indL].cy() + blobsYellow[indR].cy()) // 2
        centerX = (maxLeft + maxRight) // 2
        #centerY = blobsYellow[biggestYellowBlob].cy()
        #centerX = blobsYellow[biggestYellowBlob].cx()
        GLXPixel = -centerX + cX
        GLYPixel = -centerY + cY
        distanceYellow = sqrt(GLXPixel * GLXPixel + GLYPixel * GLYPixel) # pixels
        #print("Yellow: " + str(distanceYellow))
        distanceYellow = realDistance(distanceYellow, goalCoords)   #cm
        #print("GLX Pixel: " + str(GLXPixel) + " GLY Pixel: " + str(GLYPixel) + " DistanceYellow: " + str(distanceYellow))
        #print("YLX: " + str(int(realDistance(GLXPixel, goalCoords))) + " YLY: " + str(int(realDistance(GLYPixel, goalCoords))), end = ' ')
        #print(GLYPixel)
        angle = atan2(GLYPixel, GLXPixel)
        #angle = (angle*57)//1
        angleYellow = angle
        img.draw_cross(centerX,centerY, (0, 255, 0))
        Y_X_CM = int(distanceYellow * cos(angleYellow))
        Y_Y_CM = int(distanceYellow * sin(angleYellow))
        #print((GLXPixel + 139) // 2)
        yGoalX = (Y_X_CM + 240) // 2
        yGoalY = (Y_Y_CM + 240) // 2
       # print("YX: " + str(Y_X_CM) + " YY: " + str(Y_Y_CM), end = ' ')
    except: pass
#----------------------------------------------

#---Blue-Goal-Values-------------------------
    centerYBlue = -1
    centerXBlue = -1
    GLYBlue = -1
    GLXBlue = -1
    angleBlue = -1
    lineBlue = -1
    sideBlue = -1
    distanceBlue = -1
#--------------------------------------------

#---Blue-Goal-Finder-------------------------
    blobsPreviousMas = 0
    biggestBlueBlob = -1
    countBlue = 0
    blobsBlue=[]
    for blueBlob in img.find_blobs([threshold_blue],roi=(0,0,319,239), pixels_threshold=150, area_threshold=150, merge=True, margin = 15):
        blobsBlue.append(blueBlob)
        if(blueBlob.area() > blobsPreviousMas):
            biggestBlueBlob = countBlue
            blobsPreviosMas = blueBlob.area()
        countBlue+=1
    #try:
        #img.draw_rectangle(blobsBlue[biggestBlueBlob].rect())
        #img.draw_cross(blobsBlue[biggestBlueBlob].cx(),blobsBlue[biggestBlueBlob].cy(), 6)
    #except: pass
    maxLeft = 320
    maxRight = 0
    count = 0
    indL = 0
    indR = 0
    centerY = 0
    centerX = 0
    #print(blobsBlue)
    for blooo, i in enumerate(blobsBlue):
        if (maxLeft > blobsBlue[blooo].x()):
            maxLeft = blobsBlue[blooo].x()
            indL = count
        if (maxRight < blobsBlue[blooo].cx() + blobsBlue[blooo].cx() - blobsBlue[blooo].x()):
            maxRight = blobsBlue[blooo].cx() + blobsBlue[blooo].cx() - blobsBlue[blooo].x()
            indR = count
        count+=1

    try:
        centerY = (blobsBlue[indL].cy() + blobsBlue[indR].cy()) // 2
        centerX = (maxLeft + maxRight) // 2
        GLXPixel = -centerX + cX
        GLYPixel = -centerY + cY
        #kekBX = int(realDistance(GLXPixel, goalCoords))
        #kekBY = int(realDistance(GLYPixel, goalCoords))
        distanceBlue = sqrt(GLXPixel * GLXPixel + GLYPixel * GLYPixel) # pixels
        #print("Blue: " + str(distanceBlue))
        distanceBlue = realDistance(distanceBlue, goalCoords)   #cm
        img.draw_cross(centerX,centerY, (0, 255, 0))
        angle = atan2(GLYPixel, GLXPixel)
        #angle = (angle*57)//1
        angleBlue = angle

        BL_X_CM = int(distanceBlue * cos(angleBlue))
        BL_Y_CM = int(distanceBlue * sin(angleBlue))
        #print((GLXPixel + 139) // 2)
        bGoalX = (BL_X_CM + 240) // 2
        bGoalY = (BL_Y_CM + 240) // 2
      #  print("BX: " + str(BL_X_CM) + " BY: " + str(BL_Y_CM), end = ' ')

    except: pass
#----------------------------------------------
    #print("bGoal" +  "YellowDistance: " + str(distanceYellow) + " BlueDistance: " + str(distanceBlue))
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
    biggestBallBlob = -1
    countBall =0
    blobsBall = []
    for ballBlob in img.find_blobs([threshold_ball],roi=(0,0,319,239), pixels_threshold=3, area_threshold=3, merge=True, margin = 3):
        if not(abs(ballBlob.cx() - cX) < 28 and abs(ballBlob.cy() - cY) < 28):
            blobsBall.append(ballBlob)
        if(ballBlob.area() > blobsPreviousMas):
            biggestballBlob = countBall
            blobsPreviosMas = ballBlob.area()
        countBall+=1
    try:
        #img.draw_rectangle(blobsBall[biggestBallBlob].rect())
        img.draw_cross(blobsBall[biggestBallBlob].cx(),blobsBall[biggestBallBlob].cy(), (255,255,255))
    except: pass


    try:

        centerY = blobsBall[biggestBallBlob].cy()
        centerX = blobsBall[biggestBallBlob].cx()
        GLXPixel = -centerX + cX
        GLYPixel = -centerY + cY
        distance = sqrt(GLXPixel * GLXPixel + GLYPixel * GLYPixel) # pixels
        distance = realDistance(distance, ballCoords)   #cm
        #print("GLXPixel: " + str(GLXPixel) + " GLYPixel: " + str(GLYPixel) + " Distance: " + str(distance))
        #print("BLX: " + str(int(realDistance(GLXPixel, ballCoords))) + " BLY: " + str(int(realDistance(GLYPixel, ballCoords))))
        angle = atan2(GLYPixel, GLXPixel)
        #angle = (angle*57)//1
        angleBall = angle
        B_X_CM = int(distance * cos(angleBall))
        B_Y_CM = int(distance * sin(angleBall))
        #print(GLXPixel)
        ballX = (B_X_CM + 240) // 2
        ballY = (B_Y_CM + 240) // 2
       # print("BAllx: " + str(B_X_CM) + " BallY: " + str(B_Y_CM), end = ' ')


    except: pass


    objects = (countBall > 0) + ((countYellow > 0) << 1) + ((countBlue > 0) << 2)
    #print(objects)
    spi = SPI(2, SPI.SLAVE, polarity=0, phase=0)
    buf[0] = 0xBB
    buf[1] = 7 #msg_length
    buf[2] = objects
    buf[3] = ballX
    buf[4] = ballY
    buf[5] = yGoalX
    buf[6] = yGoalY
    buf[7] = bGoalX
    buf[8] = bGoalY
    buf[9] = crc8(buf, 9)
    spi.write(buf)
    spi.deinit()

    img.draw_cross(cX, cY, (0,255,0))
    img.draw_circle(cX,cY, 25)
    #print(objects)
    #print("Yellow: " + str(distanceYellow) + " BLue: " + str(distanceBlue))
    print(clock.fps())
