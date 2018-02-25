import json
import serial
ser = serial.Serial('/dev/tty.usbserial', 9600)

op_state_map = {
  0: 'STATE_OFF',
  1: 'STATE_TAKEOFF',
  2: 'STATE_START_FLYING',
  3: 'STATE_FLY',
  4: 'STATE_START_LANDING',
  5: 'STATE_LAND',
  6: 'STATE_STOPPED'
}

while True:
  data = json.loads(ser.readline())
  print('[' + data['t'] + '] ' + op_state_map[data['op']] + ': ' + data['is'] + ' / ' + data['ss'] + ' / ' + data['aux'])
