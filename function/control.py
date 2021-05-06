# import serial
import numpy as np 
import getch

#serial output
COM_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
# COM_PORT = '/dev/ttyTHS2'
BAUD_RATES = 115200
# BAUD_RATES = 9600
# ser = serial.Serial(COM_PORT, BAUD_RATES)
servo_adjust_array = []
speed_array = []
sos = 0

while 1:
    key = getch.getch()

    if key == "q":
        break

    if key == 'w':
        sos = 0
        speed = 30
        speed_array.append(speed)
        if servo_adjust_array:
            servo_adjust = servo_adjust_array[len(servo_adjust_array)-1]
        else:
            servo_adjust = 0

    if key == 's':
        sos = 0
        speed = 28
        speed_array.append(speed)
        if servo_adjust_array:
            servo_adjust = servo_adjust_array[len(servo_adjust_array)-1]
        else:
            servo_adjust = 0

    if key == 'a':
        sos = 0
        if speed_array:
            speed = speed_array[len(speed_array)-1]
        else:
            speed = 0
        servo_adjust = -15
        servo_adjust_array.append(servo_adjust)
    
    if key == 'd':
        sos = 0
        if speed_array:
            speed = speed_array[len(speed_array)-1]
        else:
            speed = 0
        servo_adjust = 15
        servo_adjust_array.append(servo_adjust)

    if key == 'x':
        speed_array= []
        servo_adjust_array= []
        speed = 0
        servo_adjust = 0
        sos = 1

    

    servo_sign = np.sign(servo_adjust)
    sign = "L"
    if servo_sign == 1:
            sign = "R"
    elif servo_sign == -1:
            sign == "L"
    elif servo_sign == 0:
            sign == "S"

    servo_adjust_for_send = "00"
    servo_adjust_round_10_for_send ="00"
    if 0 <= abs(servo_adjust) < 10: #single digit
            servo_adjust_for_send = "0{}".format(abs(servo_adjust))
    elif 100 > abs(servo_adjust) >= 10: #double digits
            servo_adjust_for_send = "{}".format(abs(servo_adjust))

    # if 0 <= abs(servo_adjust_round_10) < 10: #single digit
    #         servo_adjust_round_10_for_send = "0{}".format(abs(servo_adjust_round_10))
    # elif 100 > abs(servo_adjust_round_10) >= 10: #double digits
    #         servo_adjust_round_10_for_send = "{}".format(abs(servo_adjust_round_10))

    speed_for_send = "0"
    if 0 <= speed < 10: #single digit
            speed_for_send = "0{}".format(speed)
    elif 10 <= speed < 100: #double digits
            speed_for_send = "{}".format(speed)
    
    
    message_for_send_speed = "F{}\n".format(speed_for_send)
    message_for_send_servo = "{}{}\n".format(sign, servo_adjust_for_send)
    message_sos = "SOS\n"
                        
    if sos == 0:
        print("speed: ", speed)
        print("servo adjust: ", servo_adjust)
        print("Message: ", message_for_send_speed, ", ", message_for_send_servo)
        # ser.write(speed.encode())
        # time.sleep(0.03)
        # ser.write(servo.encode()) 
    else:
        # ser.write(message_sos.encode())
        print("SOS")     

