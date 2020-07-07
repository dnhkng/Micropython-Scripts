import mcp2515
import hmac
from machine import SPI
from machine import Pin
import time


can = mcp2515.CAN(SPI(1), cs=Pin('X5'))
can.Start(8,100)


def getSeed():
    requestMSG = {'ext':False,  \
                  'id':0x657, \
                  'data':b'\x8b\x02\x27\x01\x55\x55\x55\x55', \
                  'dlc':8,\
                  'rtr':False}
    seed = []
    can.Send_ms
    g(requestMSG)
    counter = 0
    while True:
        msg = can.Recv_msg()
        if msg is not None:
            if counter == 0:
                seed.extend(msg['data'][5:8])
            elif counter == 1:
                seed.extend(msg['data'][2:8])
            elif counter == 2:
                seed.extend(msg['data'][2:8])
            else:
                seed.extend(msg['data'][2:3])
                break
            counter += 1

    return bytes(seed)
            

def sendResponse(key, seed):
    response = hmac.HMAC(key, seed).out

    responseMSG = {'ext':False,  \
                   'id':0x657, \
                   'dlc':8,\
                   'rtr':False}

    responseMSG['data'] = b'\x8b\x10\x12\x27\x02' + response[:3]
    can.Send_msg(responseMSG)
    time.sleep(0.01)

    responseMSG['data'] = b'\x8b\x21' + response[3:9]
    can.Send_msg(responseMSG)
    time.sleep(0.01)

    responseMSG['data'] = b'\x8b\x22' + response[9:15]
    can.Send_msg(responseMSG)
    time.sleep(0.01)

    responseMSG['data'] = b'\x8b\x23' + response[15:] + b'\x00\x00\x00\x00\x00'
    can.Send_msg(responseMSG)

    return can.Recv_msg()

