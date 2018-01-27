import serial
import sys

def main(argv):
  com_port = "/dev/ttyACM0"
  baudrate = 115200
  ser = serial.Serial(com_port,baudrate)
  try:
    print(ser.name)
    all_inputs = map(int,sys.argv[1:])
    input = all_inputs[0]
    print input
    if(input):
      ser.write("MotorOn\n")
      ser.write("MotorOn\n")
    else:
      ser.write("MotorOff\n")
      ser.write("MotorOff\n")
  finally:
    ser.close()

if __name__ == "__main__":
  main(sys.argv[1:])
