import serial
from time import sleep
from crc16 import calcula_CRC

addr = 0x01

codes = {
  "req": 0x23,
  "res": 0x16
}

sub_codes = {
  "a1": 0xA1,
  "a2": 0xA2,
  "a3": 0xA3,
  "b1": 0xB1,
  "b2": 0xB2,
  "b3": 0xB3
}

ser = serial.Serial("/dev/ttyS0", 9600)

def write_uart(addr, code, sub_code, data):
  send_data = [addr, code, sub_code, data]
  crc = calcula_CRC(send_data, 4)
  print(crc)
  send_data.append(crc)
  #send_data = bytearray(send_data)
  print(f"send_data: {send_data}")
  ser.write(send_data)

def read_uart(addr, code, sub_code, data):
  recv_data = ser.read()
  print(recv_data)

try:
  while True:
    write_uart(addr=addr, code=codes["req"], sub_code=sub_codes["a1"], data=sub_codes["a1"])
    #recv_data = ser.read()
    sleep(0.03)
    read_uart(addr=addr, code=codes["req"], sub_code=sub_codes["a1"], data=sub_codes["a1"])
    sleep(1)
except (KeyboardInterrupt, serial.SerialException, serial.SerialTimeoutException) as error:
  print(f"\nerror: {error}")
  ser.close()
  exit(1)
