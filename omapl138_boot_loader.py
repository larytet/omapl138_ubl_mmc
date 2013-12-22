#! /usr/bin/env python
# -*- coding: utf-8 -*-


import sys, re, fileinput, os, tempfile;
from optparse import OptionParser;
from array import *
import serial, random
import traceback;
import datetime, time
import re, struct
import logging as log
import inspect 

MAGICNUMBER     = 0x41504954;
SECTION_LOAD    = 0x58535901;
REQUESTCRC      = 0x58535902;
ENABLECRC       = 0x58535903;
DISABLECRC      = 0x58535904;
JUMP            = 0x58535905;
JUMP_CLOSE      = 0x58535906;
SET             = 0x58535907;
START_OVER      = 0x58535908;
CMPSECTION_LOAD = 0x58535909;
SECTION_FILL    = 0x5853590A;

PING            = 0x5853590B;
                  
GET             = 0x5853590C;
FUNCTIONEXEC    = 0x5853590D;
FASTBOOT        = 0x58535913;
READWAIT        = 0x58535914;
FINALFXNREG     = 0x58535915;
SECUREKEYLOAD   = 0x58535920;
ENCSECTION_LOAD = 0x58535921;
SECSECTION_LOAD = 0x58535922;
SETSECEXITMODE  = 0x58535923;
SETDELEGATEKEY  = 0x58535924;
REMDELEGATEKEY  = 0x58535925;
SEQREADENABLE   = 0x58535963;
XMTSTARTWORD    = 0x58535441;
RCVSTARTWORD    = 0x52535454;



def convertToInt(s, base):
    value = None;
    try:
        value = int(s, base);
        result = True;
    except:
        log.error("Bad formed number '{0}'".format(s));
        result = False;
    return (result, value);

# Open a file for reading or writing
# Returns handle to the open file and result code False/True
def openFile(filename, flag):
    try:
        fileHandle = open(filename, flag) # read text file
    except Exception:
        log.error('Failed to open file {0}'.format(filename))
        print sys.exc_info()
        return (False, None)
    else:
        return (True, fileHandle)


def createTTY(device, rate):
    tty = serial.Serial();
    tty.port = device;
    tty.baudrate=rate;
    tty.bytesize=serial.EIGHTBITS;
    tty.parity=serial.PARITY_NONE;
    tty.stopbits=serial.STOPBITS_ONE;
    tty.timeout=10;
    tty.writeTimeout=0;
    tty.xonxoff=False;
    tty.rtscts=False;
    tty.dsrdtr=False;
    return tty

def resetTTY(tty):
    if (os.name != 'nt'):
        try:
            isOpen = tty.isOpen();
            rateOrig = tty.getBaudrate();
            if (isOpen):                 # if open close first - I can not (?) set rate for the opened I/O in Python
                tty.close();

            tty.setBaudrate(0)
            tty.open();
            time.sleep(0.05);              # minicom waits 1s
            tty.close();

            tty.setBaudrate(rateOrig)     # i am going to leave the serial in the original rate and state
            if (isOpen):
                tty.open();
        except Exception:
            traceback.print_exc();
            log.error("Failed to toggle baud rate for the I/O")

def connectTTY(device, rate):
    while (True):
        result = False;
        tty = None

        try:
            tty = createTTY(device, rate)
            resetTTY(tty);
            tty.open()
            result = True;
        except serial.SerialException:
            traceback.print_exc();
            break;
        
        break;

    return (result, tty);

def flushTTY(tty):
    try:
        tty.flush();
    except Exception:
        traceback.print_exc();

# write data represented to the tty
def writeTTY(tty, data):
    try:
        tty.flush();
        tty.write(data);
    except Exception:
        tty.flush();
        traceback.print_exc();

# read all characters, similar to flush, buit i wait for timeout
def readTTY(tty, expectedCount):
    s = ''

    try:
        s = tty.read(expectedCount)
    except Exception:
        log.error("Failed to read TTY")
        traceback.print_exc();
    
    return s

def buildhexstring(value, width=0, prefix=''):
    valueStr = hex(value)
    valueStr = valueStr.lstrip("0x")
    valueStr = valueStr.rstrip("L")
    valueStr = valueStr.upper();
    if (width > 0):
        valueStr = valueStr.zfill(width)   # add zeros the left

    valueStr = prefix + valueStr

    return valueStr


def waitBOOTME(serialDevice):
    pattern = '.*BOOTME.*';
    s = ''
    while (True):
        s = readTTY(serialDevice, 8)
        s = s.strip()
        match = re.match(pattern, s);
        if (match):
            break;
        else:
            if (len(s) > 0):
                printAsciiString(s, "I got:")
                log.debug("Not BOOTME message")
        time.sleep(0.2);

def convertWordToArray(word):
    byteArray = struct.pack("<I", word)
    return byteArray

def sendWordTTY(serialDevice, word, checkRecv=False):
    res = True
    
    byteArray = convertWordToArray(word)
    #print "Send word '{0}'".format(byteArray)
    while (True):
        writeTTY(serialDevice, byteArray)
        if (not checkRecv):
            break;
        res = checkRecvOpcode(serialDevice, word)
        
        break
        
    return res;
        
def printAsciiString(s, prefix=""):
    msg = prefix
    if (len(s) <= 0):
        msg = msg + "None"
        log.debug(msg)
    for b in s:
        c = buildhexstring(ord(b), 2)
        msg = msg + c + " "
    log.debug(msg)

def getXMTSTARTWORD(bits=8):
    return (XMTSTARTWORD >> (32-bits))

def getRCVSTARTWORD(bits=8):
    return (RCVSTARTWORD >> (32-bits))
    
def startWordSyncDevice(serialDevice):
    iobits = 8
    while True:
        sendWordTTY(serialDevice, getXMTSTARTWORD(iobits))
        s = readTTY(serialDevice, iobits/8)
        if (len(s) == iobits/8):
            received = ord(s[0])
            expected = getRCVSTARTWORD(iobits)
            if (received == expected):
                log.info("Word sync successs")
                break
            else:
                log.debug("I have got {0} instead of {1}".format(buildhexstring(received, 2), buildhexstring(expected, 2)))
        time.sleep(0.05)

def checkEchoResponse(serialDevice, request):
    maxRequestLen = 4
    res = False
    while (True):

        s = readTTY(serialDevice, maxRequestLen)
        printAsciiString(s, "I got:")
        l = len(s)
        if (l < 4):
            log.error("I got only {0} bytes in the ping response instead of 4 bytes".format(l))
            break;

        res = True
        for i in range(4):
            d1 = ord(s[i])
            d2 = (request >> (i*8) & 0xFF)
            if (d1 != d2):
                log.error("I got {0} in the byte {1} instead of {2}".format(buildhexstring(d1, 2), i, buildhexstring(d2, 2)))
                res = False
                break;
      
        break;

    return res;
    
def pingDevice(serialDevice):
    
    res = False
    count = 3
    
    while True:
        res = sendWordTTY(serialDevice, PING, True)
        if (not res):
            log.debug("No response to PING")
            break
        else:
            log.debug("I have got response to PING")
        
        sendWordTTY(serialDevice, count)
        res = checkEchoResponse(serialDevice, count)
        if (not res):
            log.debug("Ping count response is not Ok")
            break
        else:
            log.debug("Ping count response is Ok")
        
        for i in range(count):
            data = i + 1                            # device does not accept zero in the ping
            sendWordTTY(serialDevice, data)
            res = checkEchoResponse(serialDevice, data)
            if (not res):
                log.debug("Ping data response is not Ok")
                break
            
        if (not res):
            log.debug("No response to PING data")
            break;
        
        res = True;
        break;
    
 
    return res;
        
    
def request2response(request):
    response = (request & 0xF0FFFFFF) | 0x02000000;
    return response

def checkRecvOpcode(serialDevice, request):
    s = readTTY(serialDevice, 4)
    printAsciiString(s, "I got:")
    l = len(s)
    response = request2response(request);
    if (l == 4):
        d = (ord(s[3]) << 24) | (ord(s[2]) << 16)  | (ord(s[1]) << 8)  | (ord(s[0]) << 0)
        if (d == response):
            log.debug("Response {0} is Ok - equal to {1}".format(buildhexstring(d, 4), buildhexstring(response, 4)))
            return True
        else:
            log.debug("Response {0} failed - not equal to {1}".format(buildhexstring(d, 4), buildhexstring(response, 4)))
            
    return False
    
def sendMagicWord(serialDevice):
    sendWordTTY(serialDevice, MAGICNUMBER)

def sendCommand(serialDevice, command):
    log.debug("Send command: {0}".format(buildhexstring(command, 4)))
    while (True):
        res = sendWordTTY(serialDevice, command, True)
        if (res):
            break;
        time.sleep(0.1)

def sendLoadCommand(serialDevice, address, size):
    sendCommand(serialDevice, SECTION_LOAD)
    sendWordTTY(serialDevice, address)
    sendWordTTY(serialDevice, size)

def sendFile(serialDevice, f):
    data = f.read()
    serialDevice.write(data);
    log.debug("Written {0} bytes".format(len(data)))

def sendJump(serialDevice, address):
    sendCommand(serialDevice, JUMP_CLOSE)
    sendWordTTY(serialDevice, address)
    s = readTTY(serialDevice, 10)
    s = s.strip()
    pattern = ".*DONE.*"
    match = re.match(pattern, s);
    if (match):
        return True;
    else:
        log.debug(("I got '{0}' instead of 'DONE'. Jump failed").format(s))
        return False

def writeData(serialDevice, dataType, address, data):
    sendCommand(serialDevice, SET)
    sendWordTTY(serialDevice, dataType)
    sendWordTTY(serialDevice, address)
    sendWordTTY(serialDevice, data)
    sendWordTTY(serialDevice, 0)   # sleep is always zero

    print "Data write done"
    
    return True

def writeMMC_debug(serialDevice):
    s = readTTY(serialDevice, 20)
    s = s.strip()
    log.info(("> '{0}'").format(s))
    flushTTY(serialDevice)
    
def writeMMC(serialDevice, f, fileSize, startBlock):
    MMC_BLOCK_SIZE  = 512
    numBlock = (fileSize+MMC_BLOCK_SIZE)/MMC_BLOCK_SIZE;
    bytes2write = numBlock*MMC_BLOCK_SIZE;
    data = f.read()

    flushTTY(serialDevice)
    writeTTY(serialDevice, "prog")
    writeMMC_debug(serialDevice)
    
    sendWordTTY(serialDevice, 0xA1ACED66)
    sendWordTTY(serialDevice, 0)  #  address of start symbol not used
    sendWordTTY(serialDevice, numBlock)
    sendWordTTY(serialDevice, startBlock)
    sendWordTTY(serialDevice, 0)  # load address not used

    writeMMC_debug(serialDevice)

    count = 0;
    pageSize = (4*1024-1)
    for d in data:
        ba = bytearray(d)
        writeTTY(serialDevice, ba)
        count = count + 1
        if ((count & pageSize) == pageSize):
            print (count+1),'bytes'
    log.debug(("File is done - '{0}' bytes").format(len(data)))
    
    # pad until the MMC block 
    bytes2write = bytes2write - len(data)
    for i in range(0, bytes2write):
        writeTTY(serialDevice, "0")
    log.debug(("Padding is done - '{0}' bytes").format(bytes2write))

    writeMMC_debug(serialDevice)
     
        
def loadFile(serialDevice, f, fileSize, baseAddress, entryPoint):
    
    # wait for 'BOOTME'
    print 'Waiting for BOOTME - reset the board'
    waitBOOTME(serialDevice)
    log.info('Got BOOTME')

    
    while (True):
        #  Run SWS procedure
        startWordSyncDevice(serialDevice)
        
        res = pingDevice(serialDevice)
        if (not res):
            break;

        #print 'Send Magic Word'
        #sendMagicWord(serialDevice)

        log.info('Send Load command for the base address {0}'.format(buildhexstring(baseAddress, 8, '0x')))
        sendLoadCommand(serialDevice, baseAddress, fileSize)

        log.info("Send the binary data")
        sendFile(serialDevice, f)

        log.info("Send the 'jump' instruction")
        res = sendJump(serialDevice, entryPoint)
        if (not res):
            break;

        print "Load done"
        
        break

def generateAIS_SET(f_ais, address, data, sleep):
    f_ais.write(convertWordToArray(SET))
    f_ais.write(convertWordToArray(0x00000002))
    f_ais.write(convertWordToArray(address))
    f_ais.write(convertWordToArray(data))
    f_ais.write(convertWordToArray(sleep))

def generateAIS_addDebug(f_ais, data):
    generateAIS_SET(f_ais, 0x80000000, data, 0x0000000A)



def generateAIS(f, f_ais, entryPoint):
    
    
    '''
    I need something like: 
        000000: 41504954 MAGICNUMBER     
        000004: 5853590D FUNCTIONEXEC    
            ROM function index 0 (00020000) 'PLL0 Configuration                        ' args:  00120000 0000040B 
        000010: 5853590D FUNCTIONEXEC    
            ROM function index 3 (00080003) 'mDDR/DDR2 Controller Configuration        ' args:  18010001 00000002 000000C5 00134832 264A3209 3C14C722 00000492 00000000 
        000028: 5853590D FUNCTIONEXEC    
            ROM function index 5 (00050005) 'EMIFA ASYNC Configuration                 ' args:  00000000 3FFFFFFD 00000000 00000000 00000002 
        00001C: 5853590D FUNCTIONEXEC    
            ROM function index 7 (00010007) 'Power and Sleep Controller Configuration  ' args:  010D0003 
        00000C: 58535901 SECTION_LOAD    
            Address=C1080000 size=358384 bytes
        0577FC: 58535906 JUMP_CLOSE      
            Address=C1080000    
    '''
    
    # Generate AIS file
    f_ais.write(convertWordToArray(MAGICNUMBER))
    generateAIS_addDebug(f_ais, inspect.currentframe().f_lineno)

    '''
        // Disable the clock pin
        SDMMC->MMCCLK &= ~(SDMMC_MMCCLK_CLKEN_MASK);
  
        // Change the divider and re-enable the clock pin
        SDMMC->MMCCLK = (SDMMC_MMCCLK_CLKEN_MASK & (1u << SDMMC_MMCCLK_CLKEN_SHIFT)) |
                  (SDMMC_MMCCLK_CLKRT_MASK & (rate << SDMMC_MMCCLK_CLKRT_SHIFT));
    '''
    f_ais.write(convertWordToArray(FUNCTIONEXEC))
    f_ais.write(convertWordToArray(0x00010002))
    DIV4 = 0x00;
    CLKRT = 0x03;
    f_ais.write(convertWordToArray((DIV4 << 8) | CLKRT))

    f_ais.write(convertWordToArray(FUNCTIONEXEC))
    f_ais.write(convertWordToArray(0x00020000))
    f_ais.write(convertWordToArray(0x00120000))
    f_ais.write(convertWordToArray(0x0000040B))

    generateAIS_addDebug(f_ais, inspect.currentframe().f_lineno)

    f_ais.write(convertWordToArray(FUNCTIONEXEC))
    f_ais.write(convertWordToArray(0x00080003))
    f_ais.write(convertWordToArray(0x18010001)) 
    f_ais.write(convertWordToArray(0x00000002)) 
    f_ais.write(convertWordToArray(0x000000C5))
    f_ais.write(convertWordToArray(0x00134832)) 
    f_ais.write(convertWordToArray(0x264A3209))
    f_ais.write(convertWordToArray(0x3C0CC722))
    f_ais.write(convertWordToArray(0x00000492))
    f_ais.write(convertWordToArray(0x00000000))

    generateAIS_addDebug(f_ais, inspect.currentframe().f_lineno)
    
    f_ais.write(convertWordToArray(FUNCTIONEXEC))
    f_ais.write(convertWordToArray(0x00050005))
    f_ais.write(convertWordToArray(0x00000000)) 
    f_ais.write(convertWordToArray(0x3FFFFFFD)) 
    f_ais.write(convertWordToArray(0x00000000))
    f_ais.write(convertWordToArray(0x00000000)) 
    f_ais.write(convertWordToArray(0x00000002))

    generateAIS_addDebug(f_ais, inspect.currentframe().f_lineno)

    f_ais.write(convertWordToArray(FUNCTIONEXEC))
    f_ais.write(convertWordToArray(0x00010007))
    f_ais.write(convertWordToArray(0x010D0003)) 

    generateAIS_addDebug(f_ais, inspect.currentframe().f_lineno)

    data = f.read()
    f_ais.write(convertWordToArray(SECTION_LOAD))
    f_ais.write(convertWordToArray(entryPoint))
    f_ais.write(convertWordToArray(len(data)))
    f_ais.write(data)

    generateAIS_addDebug(f_ais, inspect.currentframe().f_lineno)

    f_ais.write(convertWordToArray(JUMP_CLOSE))
    f_ais.write(convertWordToArray(entryPoint))

    print "AID generation done"


def parseAIS_MAGICNUMBER        (f, offset):
    return (True, 4)

def parseAIS_SECTION_LOAD        (f, offset):
    address = parseAIS_readWord(f)
    size = parseAIS_readWord(f)
    data = f.read(size)
    if (len(data) != size):
        print "\tRead {0} bytes instead of {1}".format(len(data), size)
    print "\tAddress={0} size={1} bytes".format(buildhexstring(address,8), size)
    
    return (True, 3*4 + size)
       
def parseAIS_REQUESTCRC          (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_ENABLECRC           (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_DISABLECRC          (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_JUMP                (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_JUMP_CLOSE          (f, offset):
    address = parseAIS_readWord(f)
    print "\tAddress={0}".format(buildhexstring(address,8))
    return (True, 4+4)
       
def parseAIS_SET                 (f, offset):
    type = parseAIS_readWord(f)
    address = parseAIS_readWord(f)
    data = parseAIS_readWord(f)
    sleep = parseAIS_readWord(f)
    print "\tAddress={0}, data={1}, type={2}, sleep={3}".format(buildhexstring(address,8), buildhexstring(data,8), type, sleep)
    return (True, 4+4*4)
       
def parseAIS_START_OVER          (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_CMPSECTION_LOAD     (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_SECTION_FILL        (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_PING                (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_GET                 (f, offset):
    print "\tNot supported"
    return (False, 0)



parseAIS_FUNCTIONEXEC_ROMFUNCTION = [
(0, 'PLL0 Configuration                        ', 2), 
(1, 'PLL1 Configuration                        ', 2),
(2, 'Clock Configuration                       ', 1),
(3, 'mDDR/DDR2 Controller Configuration        ', 8),
(4, 'EMIFA SDRAM Configuration                 ', 5),
(5, 'EMIFA ASYNC Configuration                 ', 5),
(6, 'PLL and Clock Configuration               ', 3),
(7, 'Power and Sleep Controller Configuration  ', 1),
(8, 'Pinmux Configuration                      ', 3),
] 
       
def parseAIS_FUNCTIONEXEC        (f, offset):
    funcIdxAndArgsCount = parseAIS_readWord(f)
    idx = (funcIdxAndArgsCount & 0xFFFF)
    argsCountRead = ((funcIdxAndArgsCount >> 16) & 0xFFFF)
    for (romfuncIdx, romfuncName, argsCount) in parseAIS_FUNCTIONEXEC_ROMFUNCTION:
        if (idx == romfuncIdx):
            break
    
    if (idx != romfuncIdx):
        print "Unknown ROM function index {0}".format(buildhexstring(idx,8))
        return (False, 8)
    
    if (argsCountRead != argsCount):
        print "\tUnexpected number of arguments for the function {0}: {1} instead of {2}".format(romfuncName, argsCountRead, argsCount)
    
    print "\tROM function index {0} ({2}) '{1}' args: ".format(idx, romfuncName, buildhexstring(funcIdxAndArgsCount, 8)),
    for arg in range(0, argsCount):
        argValue = parseAIS_readWord(f)
        print "{0}".format(buildhexstring(argValue, 8)),
    print ""
    return (True, 4+4+argsCount*4)
        
       
def parseAIS_FASTBOOT            (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_READWAIT            (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_FINALFXNREG         (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_SECUREKEYLOAD       (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_ENCSECTION_LOAD     (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_SECSECTION_LOAD     (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_SETSECEXITMODE      (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_SETDELEGATEKEY      (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_REMDELEGATEKEY      (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_SEQREADENABLE       (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_XMTSTARTWORD        (f, offset):
    print "\tNot supported"
    return (False, 0)
       
def parseAIS_RCVSTARTWORD        (f, offset):
    print "\tNot supported"
    return (False, 0)
       

parseAIS_parsesrs = [
    (MAGICNUMBER     , "MAGICNUMBER     ", parseAIS_MAGICNUMBER       ),
    (SECTION_LOAD    , "SECTION_LOAD    ", parseAIS_SECTION_LOAD     ),   
    (REQUESTCRC      , "REQUESTCRC      ", parseAIS_REQUESTCRC       ),
    (ENABLECRC       , "ENABLECRC       ", parseAIS_ENABLECRC        ),
    (DISABLECRC      , "DISABLECRC      ", parseAIS_DISABLECRC       ),
    (JUMP            , "JUMP            ", parseAIS_JUMP             ),
    (JUMP_CLOSE      , "JUMP_CLOSE      ", parseAIS_JUMP_CLOSE       ),
    (SET             , "SET             ", parseAIS_SET              ),
    (START_OVER      , "START_OVER      ", parseAIS_START_OVER       ),
    (CMPSECTION_LOAD , "CMPSECTION_LOAD ", parseAIS_CMPSECTION_LOAD  ),
    (SECTION_FILL    , "SECTION_FILL    ", parseAIS_SECTION_FILL     ),
    (PING            , "PING            ", parseAIS_PING             ),
    (GET             , "GET             ", parseAIS_GET              ),
    (FUNCTIONEXEC    , "FUNCTIONEXEC    ", parseAIS_FUNCTIONEXEC     ),
    (FASTBOOT        , "FASTBOOT        ", parseAIS_FASTBOOT         ),
    (READWAIT        , "READWAIT        ", parseAIS_READWAIT         ),
    (FINALFXNREG     , "FINALFXNREG     ", parseAIS_FINALFXNREG      ),
    (SECUREKEYLOAD   , "SECUREKEYLOAD   ", parseAIS_SECUREKEYLOAD    ),
    (ENCSECTION_LOAD , "ENCSECTION_LOAD ", parseAIS_ENCSECTION_LOAD  ),
    (SECSECTION_LOAD , "SECSECTION_LOAD ", parseAIS_SECSECTION_LOAD  ),
    (SETSECEXITMODE  , "SETSECEXITMODE  ", parseAIS_SETSECEXITMODE   ),
    (SETDELEGATEKEY  , "SETDELEGATEKEY  ", parseAIS_SETDELEGATEKEY   ),
    (REMDELEGATEKEY  , "REMDELEGATEKEY  ", parseAIS_REMDELEGATEKEY   ),
    (SEQREADENABLE   , "SEQREADENABLE   ", parseAIS_SEQREADENABLE    ),
    (XMTSTARTWORD    , "XMTSTARTWORD    ", parseAIS_XMTSTARTWORD     ),
    (RCVSTARTWORD    , "RCVSTARTWORD    ", parseAIS_RCVSTARTWORD     ),
]

def parseAIS_getParser(command):
    for (c, name, func) in parseAIS_parsesrs:
        if (c == command):
            return (c, name, func)
    return None

def parseAIS_printCommand(offset, name, word):
    if (name != None):
        print "{0}: {2} {1}".format(buildhexstring(offset,6), name, buildhexstring(word, 8))

def parseAIS_readWord(f):
    bytes = f.read(4)
    if (len(bytes) == 0):
        return None

    word = struct.unpack('<I', bytes)[0]
    return word


def parseAIS(f):

    offset = 0
    
    while (True):
        word = parseAIS_readWord(f)
        if (word == None):
            break;

        (c, name, parser) = parseAIS_getParser(word)

        if (name != None):
            parseAIS_printCommand(offset, name, word)
            
        if ((offset == 0) and (word != MAGICNUMBER)):
            print "First word should be magic number {0}".format(buildhexstring(MAGICNUMBER, 8))
            break;

        (res, offset) = parser(f, offset)
        if (not res):
            break;
    
        

def createOptionsParser():
    # create parser for the command line options
    parser = OptionParser();

    # command line options
    parser.add_option("-d", "--device", dest="deviceName", metavar="FILE", help="Serial device name. For example, /dev/ttyUSB0", default=None);
    parser.add_option("-l", "--load", dest="loadFile", metavar="STR", type="string", help="BIN file to load, base address, entry point. For example, 'u-boot.bin 0xC1000000 0xC1000000' ", default=None);
    parser.add_option("-w", "--write", dest="writeData", metavar="STR", type="string", help="Write data to a register. Make sure that you 'init' the connection first. For example, '2 0x80000000 0xaa55aa55' ", default=None);
    parser.add_option("-i", "--init", dest="initConnection", action="store_true", default=False, help="Iinitialize the connection after power cycle");
    parser.add_option("-p", "--ping", dest="sendPing", action="store_true", default=False, help="Send ping command to the bootloader");
    parser.add_option("-m", "--mmc", dest="writeMMC", metavar="STR", type="string", default=None, help="Program specified file to the MMC. For example 'ubl.bin 25 0xC0000000', where 25 is first block to write on the MMC device, 0xc0000000 - entry point");
    parser.add_option("-a", "--ais", dest="generateAIS", metavar="STR", type="string", default=None, help="Generate AIS file for programming MMC. For example 'uboot.bin 0xC1080000 u-boot.bin.ais'");
    parser.add_option("--parse-ais", dest="parseAIS", metavar="FILE", type="string", default=None, help="Parse AIS file");
    parser.add_option("-v", "--verbose", dest="verboseOutput", action="store_true", default=False, help="Print verbose output");
    parser.add_option("--debug", dest="debugOutput", action="store_true", default=False, help="Print debug information");
    return parser;

def mainLoop():
    global verboseOutput;
    global cmdOptions;
    global fsmState;

    parser = createOptionsParser();

    # run the command line options parser
    # parse command line arguments
    (cmdOptions, args) = parser.parse_args();
    
    log.basicConfig(format="%(message)s", level=log.ERROR)
    logger = log.getLogger()
    serialDevice = None
    while True:
        if (cmdOptions.debugOutput):
            logger.setLevel(level=log.DEBUG)

        if (cmdOptions.verboseOutput) and (not cmdOptions.debugOutput):
            logger.setLevel(level=log.INFO)


        if ((serialDevice == None) and (cmdOptions.deviceName != None)):
            deviceName = cmdOptions.deviceName
            (res, serialDevice) = connectTTY(deviceName, 115200)
            if (not res):
                print "Failed to open serial device ", deviceName
                break;

        if (cmdOptions.parseAIS != None):
            generateAISStrings = cmdOptions.parseAIS.split()
            filename = generateAISStrings[0].strip()
            (res1, f) = openFile(filename, "rb"); 
            if (res1):
                fileSize = os.path.getsize(filename)
            if (res1 and fileSize):
                print "Parse AIS file {0} ".format(filename)
            parseAIS(f)
            f.close()
            break

        if (cmdOptions.generateAIS != None):
            generateAISStrings = cmdOptions.generateAIS.split()
            filename = generateAISStrings[0].strip()
            filename_ais = generateAISStrings[2].strip()
            entryPointStr = generateAISStrings[1].strip()
            (res2, f) = openFile(filename, "rb"); 
            (res3, f_ais) = openFile(filename_ais, "wb"); 
            (res1, entryPoint) = convertToInt(entryPointStr, 16)
            res4 = False
            if (res2):
                fileSize = os.path.getsize(filename)
                res4 = True
            if (res1 and res2 and fileSize and res3):
                print "Generate AIS file {0} load address {1}, size {2}".format(filename, buildhexstring(entryPoint, 8, '0x'), fileSize)
                generateAIS(f, f_ais, entryPoint)
                f.close()
                f_ais.close()
            elif (not res4 or not fileSize):
                print "File size of file {0} is zero".format(filename)
            elif (not res1):
                print "Entry point {0} is not a number".format(entryPointStr)
            elif (not res2):
                print "Failed to open file {0} for reading".format(filename)
            elif (not res3):
                print "Failed to open file {0} for writing".format(filename_ais)

            break;
        
        if (cmdOptions.deviceName == None):
            print "No serial device is specified - use -d"
            break;

        if (cmdOptions.initConnection) :
            startWordSyncDevice(serialDevice)

        if (cmdOptions.writeData != None):
            writeDataOption = cmdOptions.writeData.split()
            dataTypeStr = writeDataOption[0].strip()
            addressStr = writeDataOption[1].strip()
            dataStr = writeDataOption[2].strip()
            (res1, address) = convertToInt(addressStr, 16)
            (res2, data) = convertToInt(dataStr, 16)
            (res3, dataType) = convertToInt(dataTypeStr, 16)
            dataTypeBitField = ((dataType & 0xFF) == 3)
            res4 = ((dataType >= 0) and (dataType <= 2)) or dataTypeBitField 
            if (res1 and res2 and res3 and res4):
                dataTypeBits = (8, 16, 32)
                if (dataTypeBitField):
                    dataTypeBitsStr = "From {0} to {1}".format(((dataType>>8) & 0xFF), ((dataType>>16) & 0xFF))
                else:
                    dataTypeBitsStr = dataTypeBits[dataType]
                print "Write {0} to {1}, {2} bits".format(buildhexstring(data, 8), buildhexstring(address, 8), dataTypeBitsStr)
                writeData(serialDevice, dataType, address, data)
                break;
            if (not res1):
                print "Address '{0}' is not recognized".format(addressStr) 
            if (not res2):
                print "Data '{0}' is not recognized".format(dataStr) 
            if (not res3):
                print "Data type '{0}' is not recognized".format(dataTypeStr) 
            if (not res4):
                print "Data type should be 0 - 8 bits, 1 - 16 bits, 2 - 32 bits or 3 (bit field)".format(dataTypeStr) 
            break;
        
            
        if (cmdOptions.sendPing):
            print "Send ping"
            pingDevice(serialDevice)
            break;

        if (cmdOptions.writeMMC != None):
            writeMMCStrings = cmdOptions.writeMMC.split()
            filename = writeMMCStrings[0].strip()
            startBlockStr = writeMMCStrings[1].strip()
            (res1, startBlock) = convertToInt(startBlockStr, 10)
            (res2, f) = openFile(filename, "rb"); 
            if (res2):
                fileSize = os.path.getsize(filename)
            if (res1 and (serialDevice != None) and fileSize):
                print "Write MMC file {0} to the block {1}, size {2}".format(filename, startBlock, fileSize)
                writeMMC(serialDevice, f, fileSize, startBlock)
                f.close()

            break;


        if (cmdOptions.loadFile != None):
            loadFileStrings = cmdOptions.loadFile.split()
            filename = loadFileStrings[0].strip()
            baseAddressStr = loadFileStrings[1].strip()
            entryPointStr = loadFileStrings[2].strip()
            (res1, baseAddress) = convertToInt(baseAddressStr, 16)
            (res2, entryPoint) = convertToInt(entryPointStr, 16)
            (res3, f) = openFile(filename, "rb"); 
            if (res3):
                fileSize = os.path.getsize(filename)
            if (res1 and res2 and res3 and (serialDevice != None) and fileSize):
                print "Load file {0} to the address {1} entry point {2}, size {3}".format(filename, buildhexstring(baseAddress, 8, '0x'), buildhexstring(entryPoint, 8, '0x'), fileSize)
                loadFile(serialDevice, f, fileSize, baseAddress, entryPoint)
                f.close()
            elif (not filesize):
                print "File size is zero for", filename
            elif (not res1):
                print "Bad base address", baseAddress
            elif (not res2):
                print "Bad entry point", entryPoint
            elif (not res3):
                print "Failed to open file for reading", filename

            break;
        
        
        parser.print_help()
        break;
        


# the very first command to be executed - chop the first argument
# and forward the rest of command line arguments to the "main"
if __name__ == "__main__":
    mainLoop()
    
    
