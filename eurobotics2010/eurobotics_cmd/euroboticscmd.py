
import os,sys,atexit
import serial
from select import select
import cmd
#import pylab
from  matplotlib import pylab 
from math import *

import msvcrt

import numpy 
import shlex
import time
import math
import warnings
warnings.filterwarnings("ignore","tempnam",RuntimeWarning, __name__)

import logging
log = logging.getLogger("EuRoboticsShell")
_handler = logging.StreamHandler()
_handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
log.addHandler(_handler)
log.setLevel(1)

EUROBOTICS_PATH=os.path.dirname(sys.argv[0])


class SerialLogger:
    def __init__(self, ser, filein, fileout=None):
        self.ser = ser
        self.filein = filein
        self.fin = open(filein, "a", 0)
        if fileout:
            self.fileout = fileout
            self.fout = open(fileout, "a", 0)
        else:
            self.fileout = filein
            self.fout = self.fin
    def fileno(self):
        return self.ser.fileno()
    def read(self, *args):
        res = self.ser.read(*args)
        self.fin.write(res)
        return res
    def write(self, s):
        self.fout.write(s)
        self.ser.write(s)

def readChar(echo=True):
    "Get a single character on Windows."
    while msvcrt.kbhit():
        msvcrt.getch()
    ch = msvcrt.getch()
    while ch in b'\x00\xe0':
        msvcrt.getch()
        ch = msvcrt.getch()
    if echo:
        msvcrt.putch(ch)
    #return ch.decode()
    return ch.decode()


class Interp(cmd.Cmd):
    prompt = "EuRobotics> "
    def __init__(self, tty, baudrate=115200):
        cmd.Cmd.__init__(self)
        self.ser = serial.Serial(tty,baudrate=baudrate, timeout = 0)
        self.escape  = "\x01" # C-a
        self.quitraw = "\x02" # C-b
        self.serial_logging = False
        #self.default_in_log_file = "/tmp/eurobotics.in.log"
        #self.default_out_log_file = "/tmp/eurobotics.out.log"
        self.default_in_log_file = "tmp\eurobotics.in.log"
        self.default_out_log_file = "tmp\eurobotics.out.log"

    def do_quit(self, args):
        return True

    def do_log(self, args):
        """Activate serial logs.
        log <filename>           logs input and output to <filename>
        log <filein> <fileout>   logs input to <filein> and output to <fileout>
        log                      logs to /tmp/eurobotics.log or the last used file"""

        if self.serial_logging:
            log.error("Already logging to %s and %s" % (self.ser.filein, 
                                                        self.ser.fileout))
        else:
            self.serial_logging = True
            files = [os.path.expanduser(x) for x in args.split()]
            if len(files) == 0:
                files = [self.default_in_log_file, self.default_out_log_file]
            elif len(files) == 1:
                self.default_in_log_file = files[0]
                self.default_out_log_file = None
            elif len(files) == 2:
                self.default_in_log_file = files[0]
                self.default_out_log_file = files[1]
            else:
                print "Can't parse arguments"

            self.ser = SerialLogger(self.ser, *files)
            log.info("Starting serial logging to %s and %s" % (self.ser.filein, 
                                                               self.ser.fileout))


    def do_unlog(self, args):
        if self.serial_logging:
            log.info("Stopping serial logging to %s and %s" % (self.ser.filein, 
                                                               self.ser.fileout))
            self.ser = self.ser.ser
            self.serial_logging = False
        else:
            log.error("No log to stop")

    def do_raw(self, args):
        "Switch to RAW mode"
        #stdin = os.open("/dev/stdin",os.O_RDONLY)
        #stdout = os.open("/dev/stdout",os.O_WRONLY)

        #stdin_termios = termios.tcgetattr(stdin)
        #raw_termios = stdin_termios[:]
        
        try:
            log.info("Switching to RAW mode")

            # iflag
            # raw_termios[0] &= ~(termios.IGNBRK | termios.BRKINT | 
            #                    termios.PARMRK | termios.ISTRIP | 
            #                    termios.INLCR | termios.IGNCR | 
            #                    termios.ICRNL | termios.IXON)
            # oflag
            #raw_termios[1] &= ~termios.OPOST;
            # cflag
            #raw_termios[2] &= ~(termios.CSIZE | termios.PARENB);
            #raw_termios[2] |= termios.CS8;
            # lflag
            #raw_termios[3] &= ~(termios.ECHO | termios.ECHONL | 
            #                    termios.ICANON | termios.ISIG | 
            #                    termios.IEXTEN);

            #termios.tcsetattr(stdin, termios.TCSADRAIN, raw_termios)


            mode = "normal"
            i= 0;
            while True:
                #print i
                i += 1

                if msvcrt.kbhit(): 
                    #c = msvcrt.getch()
                    c = readChar(echo = False)                 
                    if mode  == "escape":
                        if c == self.escape:
                            self.ser.write(self.escape)
                        elif c == self.quitraw:
                            return
                        else:
                            self.ser.write(self.escape)
                            self.ser.write(c)
                    else:
                        if c == self.escape:
                            mode = "escape"
                        else:
                            self.ser.write(c)
                            
                if self.ser.inWaiting():                                       
                    #sys.stdout.write(self.ser.read())
                    #msvcrt.putch(self.ser.read())

                    data = self.ser.read(1)
                    if data == "\x1b":
                        data = self.ser.read(3)
                        if data == "[0K":
                            pass
                        else:
                            sys.stdout.write(data)
                    else:
                        sys.stdout.write(data)
                    
        finally:
            #termios.tcsetattr(stdin, termios.TCSADRAIN, stdin_termios)
            log.info("Back to normal mode")
            

if __name__ == "__main__":
    try:
        import pyreadline,atexit
    except ImportError:
        pass
    else:
        #histfile = os.path.join(os.environ["HOME"], ".eurobotics_history")
        histfile = os.path.join("eurobotics_history")
        atexit.register(pyreadline.write_history_file, histfile)
        try:
            pyreadline.read_history_file(histfile)
        except IOError:
            pass
    
    device = "COM1"
    if len(sys.argv) > 1:
        device = sys.argv[1]
    interp = Interp(device)
    while 1:
        try:
            interp.cmdloop()
        except KeyboardInterrupt:
            print
        except Exception,e:
            l = str(e).strip()
            if l:
                log.exception("%s" % l.splitlines()[-1])
            continue
        break
