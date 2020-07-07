#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

class CAN (object):
    def __init__(self, spi, cs):
        # Function: MCP2515 chip initialization
        self.spi = spi
        self.cs = cs
        self._RxBuf = []

        self.cs.init(self.cs.OUT, value=1)
        try:
            master = self.spi.MASTER
        except AttributeError:
            # on ESP8266
            self.spi.init(baudrate=8000000, phase=0, polarity=0)
        else:
            # on pyboard
            self.spi.init(master, baudrate=8000000, phase=0, polarity=0)

        # Software reset
        self._spi_Reset()
        # If you can read the data, it is considered that there is no problem with initialization. At least the chip was soldered.
        time.sleep(0.2)
        mode = self._spi_ReadReg(b'\x0E')
        if (mode == 0):
            raise OSError("MCP2515 init fail .")

    def SetINT(self, INT):
        pass  # To be added. Looking forward to a very good day!

    def Stop(self):
        # Function: stop MCP2515
        self._spi_WriteBit(b'\x0f', b'\xe0', b'\x20')  # sleep mode

    def Start(self, SpeedCfg, Filter=None, ListenOnly=False):
        # Function: start MCP2515
        # SpeedCfg: CAN communication speed
        # The supported communication rates are currently as follows:
        # 5K, 10K, 20K, 33K, 40K, 50K, 80K, 95K, 100K, 125K, 200K,  250K, 500K, 1000K
        # The valid values ​​of the corresponding parameters are:
        # 5, 10, 20, 33, 40, 50, 80, 95, 100, 125, 200, 250, 500, 1000
        # Filter: Received message filtering mode
        # To be optimized.
        # ListenOnly: Whether to specify the listening mode
        
        # Set to configuration mode
        self._spi_Reset()
        self._spi_WriteBit(b'\x0f',b'\xe0',b'\x80')
        # Set the communication rate
        SpeedCfg_at_16M = {\
            1000: b'\x82\xD0\x00', \
            500: b'\x86\xF0\x00', \
            250: b'\x85\xF1\x41', \
            200: b'\x87\xFA\x01', \
            125: b'\x86\xF0\x03', \
            100: b'\x87\xFA\x03', \
            95: b'\x07\xAD\x03', \
            80: b'\x87\xFF\x03', \
            50: b'\x87\xFA\x07', \
            40: b'\x87\xFF\x07', \
            33: b'\x07\xBE\x09', \
            20: b'\x87\xFF\x0F', \
            10: b'\x87\xFF\x1F', \
            5: b'\x87\xFF\x3F'}
        cfg = SpeedCfg_at_16M.get(SpeedCfg,(b'\x00\x00\x00'))
        self._spi_WriteReg(b'\x28',cfg)
        del SpeedCfg_at_16M
        # Channel 1 packet filtering settings
        if (Filter==None):
            self._spi_WriteBit(b'\x60',b'\x64',b'\x64')
        else:
            self._spi_WriteBit(b'\x60',b'\x64',b'\x04')
            self._spi_WriteReg(b'\x00',Filter.get('F0'))
            self._spi_WriteReg(b'\x04',Filter.get('F1'))
            self._spi_WriteReg(b'\x20',Filter.get('M0'))
        # Disable channel 2 message reception
        self._spi_WriteBit(b'\x70',b'\x60',b'\x00')
        self._spi_WriteReg(b'\x08',b'\xff\xff\xff\xff')
        self._spi_WriteReg(b'\x10',b'\xff\xff\xff\xff')
        self._spi_WriteReg(b'\x14',b'\xff\xff\xff\xff')
        self._spi_WriteReg(b'\x18',b'\xff\xff\xff\xff')
        self._spi_WriteReg(b'\x24',b'\xff\xff\xff\xff')
        # Set to normal mode or monitor mode
        mode = b'\x00' if (ListenOnly==False) else b'\x60'
        self._spi_WriteBit(b'\x0f',b'\xe0', mode)

    def Send_msg(self, msg, sendchangel=None):
        # Function: send messages.
        # msg:
        # msg['id']: ID of the message to be sent
        # msg['ext']: Whether the message to be sent is an extended frame
        # msg['data']: The data of the message to be sent
        # msg['dlc']: Length of the message to be sent
        # msg['rtr']: Whether the message to be sent is a remote frame
        # sendchangel:
        # Specify the channel to send packets. Valid values ​​are as follows:
        # 0: channel 0
        # 1: channel 1
        # 2: channel 2
        # MCP2515 provides three sending channels, and channel 0 is used by default. Follow up to automatically find free channels, so stay tuned.
        # Note: If there is a message to be sent in the channel, the previous message will be stopped.
        # Then replace it with a new message and enter the pending state again.
        if sendchangel==None:
            sendchangel = 0
        self._MsgVerificationCheck(msg) # msg check.
        # Stop sending messages in the previous register
        ctl = (((sendchangel%3)+3)<<4).to_bytes(1,'big')
        self._spi_WriteBit(ctl,b'\x08',b'\x00')
        # Data regularization
        self.TxBuf = bytearray(13)
        if msg.get('ext'):
            self.TxBuf[0] = ((msg.get('id')) >> 21) & 0xFF
            id_buf = ((msg.get('id')) >> 13) & 0xE0
            id_buf |= 0x08
            id_buf |= ((msg.get('id')) >> 16) & 0x03
            self.TxBuf[1] = id_buf
            self.TxBuf[2] = ((msg.get('id')) >> 8) & 0xFF
            self.TxBuf[3] = (msg.get('id')) & 0xFF
            if msg.get('rtr'):
                self.TxBuf[4] |= 0x40
        else:
            self.TxBuf[0] = ((msg.get('id')) >> 3) & 0xFF
            self.TxBuf[1] = ((msg.get('id')) << 5) & 0xE0
            if msg.get('rtr'):
                self.TxBuf[1] |= 0x10
        if msg.get('rtr')==False:
            self.TxBuf[4] |= msg.get('dlc') & 0x0F
            self.TxBuf[5:13] = msg.get('data')[:msg.get('dlc')]
        # Data loading
        dat = ((((sendchangel%3)+3)<<4)+1).to_bytes(1,'big')
        self._spi_WriteReg(dat,self.TxBuf)
        # Send
        self._spi_SendMsg(1<<sendchangel) # self._spi_WriteBit(ctl,b'\x08',b'\x08')

    def Recv_msg(self):
        # Function: Inquire whether the MCP2515 has received a message. If there is, it is stored in Buf. CheckRx is called.
        # Query whether Buf has a message. If yes, return the earliest received frame, otherwise return None.
        # Return to Msg description:
        # msg['tm']: Time to receive the message. Timer started at power on. unit=1ms.
        # msg['id']: ID of the received message
        # msg['ext']: Whether the received message is an extended frame
        # msg['data']: received message data
        # msg['dlc']: Length of received message
        # msg['rtr']: Whether the received message is a remote frame
        # Note: Only one frame of message is returned at a time, one frame!
        self.CheckRx()
        if len(self._RxBuf) == 0: 
            return None
        dat = self._RxBuf.pop(0)
        msg = {}
        msg['tm'] = int.from_bytes(dat[-8:],'big')
        msg['dlc'] = int.from_bytes(dat[4:5],'big') & 0x0F
        msg['data'] = dat[5:13]
        ide = (int.from_bytes(dat[1:2],'big')>>3) & 0x01 # 0: standard frame 1: extended frame
        msg['ext'] = True if ide==1 else False
        id_s0_s10 = int.from_bytes(dat[:2],'big') >> 5
        id_e16_e17 = int.from_bytes(dat[:2],'big') & 0x03
        id_e0_e15 = int.from_bytes(dat[2:4],'big')
        if msg['ext']:
            msg['id'] = (id_s0_s10<<18) + (id_e16_e17<<16) + id_e0_e15
            msg['rtr'] = True if (int.from_bytes(dat[4:5],'big') & 0x40) else False
        else:
            msg['id'] = id_s0_s10
            msg['rtr'] = True if (int.from_bytes(dat[1:2],'big') & 0x10) else False
        return msg

    def CheckRx(self):
        # Function: Inquire whether the MCP2515 has received a message. If there is, save Buf and return TRUE, otherwise return False.
        # Note: Failure to store the messages in the MCP in Buf in time may cause the MCP to fail to receive new messages.
        # In other words, the message may be lost.
        # So, call this function as much as possible~~
        rx_flag = int.from_bytes(self._spi_ReadStatus(),'big')
        if (rx_flag&0x01):
            dat = self._spi_RecvMsg(0)
            tm = (time.ticks_ms()).to_bytes(8,'big')
            self._RxBuf.append(dat+tm)
        if (rx_flag&0x02):
            dat = self._spi_RecvMsg(1)
            tm = (time.ticks_ms()).to_bytes(8,'big')
            self._RxBuf.append(dat+tm)
        return True if (rx_flag&0b11000000) else False

    def Pin_RXxBF_as_Output(self, pin, value=None):
        # Function: Set or get the output status of the RXxBF port.
        # pin: the set port
        # value:
        # Empty: Returns the status of the port. 1--High level 0--Low level
        # 0: Set the port to low level
        #1: Set the port to high level
        # Other: invalid
        # Note: After calling this method, even if the port was originally in the receive message interrupt mode, the port will be set to output.
        self._spi_WriteBit(b'\x0c',b'\x0F',b'\x0C')
        if value==None:
            reg = self._spi_ReadReg(b'\x0c')
            v = int.from_bytes(reg,'big') & {0:0x10,1:0x20}.get(pin,0x00)
            return 1 if (v!=0) else 0
        p = {0:b'\x10', 1:b'\x20'}.get(pin,b'\x00')
        v = {'H':b'\x30','L':b'\x00'}.get(value,b'\x00')
        self._spi_WriteBit(b'\x0c',p,v)

    def Pin_TXxRTS_as_Input(self, pin):
        # Function: Set and get the output status of TXxRTS port.
        # pin: the set port
        # value:
        # Empty: Returns the status of the port. 1--High level 0--Low level
        # Other: invalid
        # Note: After calling this method, the port will be set to input even if the port was originally in the interrupt mode for sending packets.
        p = {2:0x20, 1:0x10, 0:0x08}.get(pin) #Port is invalid? Don't care!
        rst = self._spi_ReadReg(b'\x0d')
        return 1 if ((rst[0]&p) != 0) else 0

    def _MsgVerificationCheck(self, msg):
        # Function: Check whether the content of msg is a CAN data format that meets the sending requirements.
        # If msg does not conform to the format, an exception is thrown.
        if not (isinstance(msg.get('ext'),bool)):
            raise Exception('Msg format error: ext logo is not of type bool.')
        if not (isinstance(msg.get('id'),int)):
            raise Exception('Msg format error: id is not of type int.')
        if not (isinstance(msg.get('data'),bytes)):
            raise Exception('Msg format error: data is not of type bytes.')
        if not (isinstance(msg.get('dlc'),int)):
            raise Exception('Msg format error: dlc is not of type int.')
        if not (isinstance(msg.get('rtr'),bool)):
            raise Exception('Msg format error: rtr is not of type bool.')
        if ((msg.get('ext') == False) and (msg.get('id')> 0x7ff)):
            raise Exception('Msg format error: id exceeds the maximum value of the standard frame.')
        if ((msg.get('ext') == True) and (msg.get('id')> 0x1fffffff)):
            raise Exception('Msg format error: id exceeds the maximum value of the extended frame.')
        if (len(msg.get('data', b'')) <msg.get('dlc')): 
            raise Exception('Msg format error: data is shorter than the length defined by dlc.')
        if (msg.get('dlc')> 8): 
            raise Exception('Msg format error: dlc length cannot be greater than 8.')

    def _spi_Reset(self):
        # Function: MCP2515_SPI instruction-reset
        self.cs.off()
        self.spi.write(b'\xc0')
        self.cs.on()
    
    def _spi_WriteReg(self, addr, value):
        # Function: MCP2515_SPI instruction-write register
        self.cs.off()
        self.cs.off()
        self.spi.write(b'\x02')
        self.spi.write(addr)
        self.spi.write(value)
        self.cs.on()

    def _spi_ReadReg(self, addr, num=1):
        # Function: MCP2515_SPI instruction-read register
        self.cs.off()
        self.spi.write(b'\x03')
        self.spi.write(addr)
        buf = self.spi.read(num)
        self.cs.on()
        return buf

    def _spi_WriteBit(self,addr, mask, value):
        # Function: MCP2515_SPI instruction-bit modification
        self.cs.off()
        self.spi.write(b'\x05')
        self.spi.write(addr)
        self.spi.write(mask)
        self.spi.write(value)
        self.cs.on()

    def _spi_ReadStatus(self):
        # Function: MCP2515_SPI instruction-read status
        self.cs.off()
        self.spi.write(b'\xa0')
        buf = self.spi.read(1)
        self.cs.on()
        return buf

    def _spi_RecvMsg(self, select):
        # Function: MCP2515_SPI instruction-read Rx buffer
        self.cs.off()
        if select==0:
            self.spi.write(b'\x90')
            buf = self.spi.read(13)
        if select==1:
            self.spi.write(b'\x94')
            buf = self.spi.read(13)
        self.cs.on()
        return buf

    def _spi_SendMsg(self, select):
        # Function: MCP2515_SPI instruction-request to send a message
        self.cs.off()
        self.spi.write((0x80+(select&0x07)).to_bytes(1,'big'))
        self.cs.on()