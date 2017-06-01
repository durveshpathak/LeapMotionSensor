import os, sys, inspect, thread, time, serial
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

ser=serial.Serial('/dev/tty.usbmodem1422',115200)
print ser.name  + "Connected To FRDM K64"

Command =str(input("Motor Command"))
if (Command =="x"):
	ser.write('x')
elif (Command =="s"):
	for i in range (0,10000):
            ser.write('s')	
            print "."


def main():
	class SampleListener(Leap.Listener):
		
		# when sensor connected -  invoke when sensor connected 
		def on_connect(self, controller):
			print "Connected"
			controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)


		def on_frame(self, controller):
			#count = count + 1
			frame = controller.frame()
			hand = frame.hands.rightmost
			rotation_y = hand.palm_position
			for hand in frame.hands:
				handType = "left hand" if hand.is_left else "right hand"
				print handType + "palm position : "
				Throttle = str(int(hand.palm_position[1]))
				print Throttle
				ser.write(Throttle)
				print " "

		
				normal = hand.palm_normal
				direction = hand.direction

				pitch = direction.pitch*Leap.RAD_TO_DEG
				roll = normal.roll*Leap.RAD_TO_DEG
				yaw = direction.yaw*Leap.RAD_TO_DEG

				print "Pitch : " + str(pitch) + "   Roll : "+str(roll)+   " Yaw : "+str(yaw)
				
				if (pitch < -15 and roll> 15):
					print "forward Left"
					ser.write('Q')

				elif (pitch < -15 and roll <-15):
					print "forward Right"
					ser.write('E')	

				elif (pitch >15 and roll <-15):
				    print " backward Right"
				    ser.write('C')

				elif (pitch >15 and roll > 15):   		
					print "backward left"
					ser.write('Z')

				elif pitch < -15:
					print "Forward"
					ser.write('F')

				elif pitch >15:
					print "Backward"
					ser.write('B')	
				elif roll <-15:
					print "Right"
					ser.write('R')	

				elif roll > 15:
					print "Left"
					ser.write('L') 

				elif yaw >15:
					print "Rudder Right"
					ser.write('>')		
				elif yaw <-15:
					print "Rudder Left"
					ser.write('<')	
				elif (pitch > -15 and pitch<15 and roll >-15 and roll < 15):
					print "hover"	
					ser.write('H')	
	listener = SampleListener()
	controller = Leap.Controller()

	# Have the sample listener receive events from the controller
	controller.add_listener(listener)

	# Keep this process running until Enter is pressed
	
	# Keep this process running until Enter is pressed
        print "Press Enter to quit..."
        try:
            sys.stdin.readline()
        except KeyboardInterrupt:
            pass
        finally:
        # Remove the sample listener when done
            controller.remove_listener(listener)


if __name__ == "__main__":
    main()

