from collections import deque
from simple_pid import PID   #da bi mogli da koristimo PID u Python-u moramo da ga importujemo iz biblioteke simple_pid
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils  #paket koji sadrzi niz OpenCV funkcija za obradu slike(rotacije slike,promena velicine slike...)
import time
from RPIO import PWM  #upraavljanje servo motorima preko PWM(pulse width modulation) signala--u nasem kodu koristicemo PWM 	 #iz biblioteke RPIO--moguce je raditi i sa GPIO.PWM

#sledece tri linije koda predstavljaju inicijalizaciju PWM-a
PWM.setup()
#inicijalizacija kanala--za razlicite servo motore koristicemo posebne kanale 
#za servoPIN1--setujemo servoMotor1 na kanal 0 ,a perioda signala je 10000(odnosno frekvencija signala je 100Hz)
PWM.init_channel(0,10000)
#za servoPIN2--setujemo servoMotor2 na kanal 1
PWM.init_channel(1,10000) 
#Napomena: Mozda je moguce setovati oba servo-motora na jedan PWM kanal

servoPIN1 = 17  #servo-motor 1 povezujemo na GPIO pin 17 Raspberry Pi
servoPIN2 = 27  #servo-motor 2 povezujemo na GPIO pin 27
 
#motori mogu da idu od 38 do 70
start1 = 51 #centar za motor donji
start2 = 49 #centar za motor desni
 
#pozicije (koordinate )centra u nasem primeru su (96.5,61.5) 
setpointX = 96.5 #centar X
setpointY = 61.5 #centarY

#koristimo PID regulatore sa vec odredjenim i kroz simulaciju proverenim vrednosima za PID
#poslednji parametar u funkciji je pozicija na kojoj zelimo da postavimo lopticu 
pidX = PID(1,0.1,0.05,setpoint = setpointX) 
pidY = PID(1,0.1,0.05,setpoint = setpointY)


ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())

#kroz promenljive colorLower i colorUpper zadajemo u kom opsegu boja moze biti loptica--kako bi je pronasli na video stream-u
colorLower = (20, 50, 50)
colorUpper = (200,255,255)
pts = deque(maxlen=args["buffer"])

if not args.get("video", False):
    vs = VideoStream(src=0).start()  #pokretanje Video stream-a--promenljivu vs koristimo za pristup video stream-u
 
else:
    vs = cv2.VideoCapture(args["video"])
 
time.sleep(2.0)


while True:
    frame = vs.read() #ucitavanje frame-ova
    frame = frame[1] if args.get("video", False) else frame
    if frame is None:
        break
        
    frame = imutils.resize(frame, width=200)  #promena velicine frame-a na 200x200 px
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	
	#sledeci deo koda pronalazi lopticu u frame-u i njenu poziciju pamti u promenljivoj cnts
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    c=len(cnts)

    #u if granu ulazimo ako pronadjemo lopticu na slici u suprotnom ulazimo u else granu
    if len(cnts) > 0:
	
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        if radius > 4:
            
			xn = x-setpointX 
            yn = y-setpointY
			#uradicemo smo P regulaciju
			xn = xn / setpointX 
            yn = yn / setpointY
			# xn i yn su u opsegu od 0 do 1
            print("X :",int(x))
            print(" Y :",int(y))
            print("Xn :",xn)
            print(" Yn :",yn)
			
            if (xn > 0.3)or(yn>0.3) :
				#servo motori mogu primiti impulse u opsegu od 38 do 70
				if ( xn>0 and yn<0 )or( xn<0 and yn>0 ) :  #ako se loptica nalazi u drugom i cetvrtom kvadrantu 
                
                    PWM.add_channel_pulse(0,17,0,int(-xn*5)+start1)
                    PWM.add_channel_pulse(1,27,0,int(-yn*5)+start2)
                else: 
                    PWM.add_channel_pulse(0,17,0,int(xn*5)+start1)
                    PWM.add_channel_pulse(1,27,0,int(yn*5)+start2)
                 
            else:      #slucaj kada je loptica blizu centra
				if ( xn>0 and yn<0 )or( xn<0 and yn>0 ) :  #ako se loptica nalazi u drugom i cetvrtom kvadrantu 
                    PWM.add_channel_pulse(0,17,0,int(-xn*3)+start1)
                    PWM.add_channel_pulse(1,27,0,int(-yn*3)+start2)
                else: 
                    PWM.add_channel_pulse(0,17,0,int(xn*3)+start1)
                    PWM.add_channel_pulse(1,27,0,int(yn*3)+start2)

     
            cv2.circle(frame, (int(x), int(y)), int(2),
                           (0, 255, 255), 3)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    else:
        PWM.add_channel_pulse(0,17,0,start1)
        PWM.add_channel_pulse(1,27,0,start2)
        PWM.clear_channel_gpio(0,17)
        PWM.clear_channel_gpio(1,27)
            
    pts.appendleft(center)


    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord("q"):
        PWM.cleanup()
        break
        
 
if not args.get("video", False):
    vs.stop()
 
else:
    vs.release()
	
    PWM.cleanup()
cv2.destroyAllWindows()