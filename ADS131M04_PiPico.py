from machine import SPI, Pin
from machine import Pin, Signal
from machine import Pin, PWM
from time import sleep
from machine import UART, Pin
from array import *
import rp2


Channel_Sum = 0
loop_count = 0

terminator = b'\xff\xff\xff'                                        # this is the buffer that is call to end a uart transmit to the display


uart1 = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))                # initiate UART1 
command = str.encode('t0.txt=\"Voltage\"')                          # buffer for the text lable on called cammand that is on page 0
uart1.write(command + terminator)                                   # writes to display Voltage in the t0 box and ends the message with the terminator 


Ch1_array = array('i',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])


spi = SPI(0,                                                         # Using SPI0 on Rpi Pico
          baudrate=8_000_000,                                       # Set baudrate to 8 Mhz
          polarity=0,                                                # Set polarity to 0
          phase=1,                                                   # set phase to 1
          bits=8,                                                    # sets bits to 8
          firstbit=SPI.MSB,                                          # specifies Most Significant bit first
          sck=Pin(18),                                               # sets SCK to Pin 18
          mosi=Pin(19),                                              # sets MOSI to Pin 19
          miso=Pin(16))                                              # sets MISO to Pin 16


cs = Pin(17, mode=Pin.OUT, value=1)                                  # Create chip-select on pin 17. and set it to high
drdy = Pin(20, Pin.IN, Pin.PULL_UP)                                  # define DRDY pin as low on pin 20
reset = Pin(21, mode = Pin.OUT, value=1)                             # define RESET pin as HIGH on pin 21


CLKIN = PWM(Pin(22))                                                 # sets CLKIN to GPIO pin 22
CLKIN.freq(8000000)                                                  # sets CLKIN frequency to 8 Mhz
CLKIN.duty_u16(32767)                                                # sets CLKIN duty cycly to 50% (65535/2=32767.5)


def RESET():#________________________________________________________# define the reset fucntion
    reset(0)                                                         # sets reset Pin to low
    sleep(0.003)                                                     # keeps pin low for 0.3ms to preform the reset on adc
    reset(1)                                                         # set reset Pin to high to complete the reset task


def Clock_config():#________________________________________________# Clock configuration funtion definition
# Register 0x03 (CLOCK) definition
 #--------------------------------------------------------------------------------------------------------------------------------------------
 # Current Configuration
 # |   0    |   0    |   0    |   0    |   0    |   0    |   1    |   0    |   0   |   0   |   0   |   1   |   1   |   1   |   1   |   1   |
 # -------------------------------------------------------------------------------------------------------------------------------------------
 # | Bit 15 | Bit 14 | Bit 13 | Bit 12 | Bit 11 | Bit 10 | Bit 9  | Bit 8  | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 # -------------------------------------------------------------------------------------------------------------------------------------------
 # |    Reserved always reads 0000     | CH3_EN | CH2_EN | CH1_EN | CH0_EN | RESERVED[2:0] |   0   |        OSR[2:0]       |     PWR[1:0]  |
 # -------------------------------------------------------------------------------------------------------------------------------------------
 
 #  NOTE 1: Bits 12 through 15 are RESERVED on the ADS131M04.
    cs(0)                                                           # set Chip Select Low
    spi.read(1, 0b0110_0001)                                        #  |  WREG command to the clock register
    spi.read(1, 0b1000_0000)                                        #  |  and only write to one register
    spi.read(1, 0x00)                                               # finish the 24 bit word
    
    spi.read(1, 0b0000_0010)                                        #  | new configuration on clock register (first byte)
    spi.read(1, 0b0001_1111)                                        #  | second byte of clock register configuration (second byte)
    spi.read(1, 0x00)                                               #  (third byte) finish the 24 bit word
    cs(1)                                                           #set Chip Select High
    
    
def Read_reg():#____________________________________________________# Read Register function definitiom
    cs(0)                                                           # Chip Select low
    spi.read(1, 0b1010_0001)                                        # | Read Register command reading the clock register (first byte)
    spi.read(1, 0b1000_0000)                                        # | read only one register (second byte)
    spi.read(1,0x00)                                                # | zeros to finish 24 bit word (third byte)
    spi.read(15)                                                    # | reading 15 bytes to complete the standard 6 word frame
    # this function still needs work and will step into a stand by command
    cs(1)                                                           # Chip Select high
    
     
def from_bytes(bytes, byteorder='big', signed=False):#______________# this function defines 2's complement
    if byteorder == 'little':
        little_ordered = list(bytes)
    elif byteorder == 'big':
        little_ordered = list(reversed(bytes))
    else:
        raise ValueError("byteorder must be either 'little' or 'big'")

    n = sum(b << i*8 for i, b in enumerate(little_ordered))
    if signed and little_ordered and (little_ordered[-1] & 0x80):
        n -= 1 << 8*len(little_ordered)

    return n                                                         # returns the signed integer
        
        
RESET()                                                              # calls the RESET funtcion to reset pin before collection data
Clock_config()                                                       # calls the Clock_config fuction 

#____________________________________________________________________________________________________________________________________
while True:                                                          # Main loop
          #(channel 1 is the only channel outputing data due to clock register configuration)
    
    cs(0)                                                            #set CS Low    
    cmd_1 = spi.read(3, 0x00)                                        # Cammand word        (Outputs a byte array)
    ch0_1 = spi.read(3, 0x00)                                        # data from channel 0 (Outputs a byte array)
    ch1_1 = spi.read(3, 0x00)                                        # data fron channel 1 (Outputs a byte array)
    ch2_1 = spi.read(3, 0x00)                                        # data from channel 2 (Outputs a byte array)
    ch3_1 = spi.read(3, 0x00)                                        # data from channel 3 (Outputs a byte array)
    crc   = spi.read(3, 0x00)                                        # CRC word (this is not being used (Outputs a byte array)) 
    cs(1)                                                            #set CS High
    
    #sleep(0.004)                                                     # sleep for 4 miliseconds
    #CH0_1 = from_bytes(ch0_1, 'big', True)                           # change channel 0 data to a float
    CH1_1 = from_bytes(ch1_1, 'big', True)                            # change channel 1 data to a float
    #CH2_1 = from_bytes(ch2_1, 'big', True)                           # change channel 2 data to a float
    #CH3_1 = from_bytes(ch3_1, 'big', True)                           # change channel 3 data to a float
    
#__________________________________________________________________________________________________________________________________________________________________
    #CH0 = int(CH0_1 * 13200000/ 8388607)                             # applies a conversion constant achieve the correct output as an interger for channel 0
    CH1 = int(CH1_1 * (13230000 / 8388607))                           # applies a conversion constant achieve the correct output as an interger for channel 1
    #CH2 = (CH2_1 * 0.00000014305116451396519112172020932677)         # applies a conversion constant achieve the correct output as an interger for channel 2
    #CH3 = (CH3_1 * 0.00000014305116451396519112172020932677)         # applies a conversion constant achieve the correct output as an interger for channel 3
    
    Ch1_array[loop_count]=CH1
    ch1_sum = sum(Ch1_array)                                          # adds all the values in the Ch1_array together
    avg = int(ch1_sum/60)                                             # this averages 60 outputs that are stored in the CH1_array
    
    
    
    
    CH0T = str.encode('x0.val=' + str(avg))                          # configures channel 1 data for uart output
    uart1.write(CH0T + terminator)                                   # writes channel 1 data to x0 block

    #print("ch0: ",CH0)                                              # print out data form channel 0 
    print("ch1: ",CH1 )                                              # print out data form channel 1
    #print("ch2: ",CH2)                                              # print out data form channel 2
    #print("ch3: ",CH3)                                              # print out data form channel 3
       
    if loop_count < 59:                                              # keeps a loop count from 0 to 59 to send an output to the Ch1_array
        loop_count = int(1 + loop_count)
    else:
        loop_count = 0    
    

