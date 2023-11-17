"""
CoolTerm Module
===============

    A Python implementation of a CoolTerm Remote Control Socket Client

    Requires Python 3

    Roger Meier, v1.4, October 2022


Create a new CoolTermSocket as follows:

    s = CoolTerm.CoolTermSocket()

Alternatively, you may also specify the host IP and Port as follows

    s = CoolTerm.CoolTermSocket("127.0.0.1", 51413)

To disconnect the socket from CoolTerm, use the following

    s.Close()


CoolTermsSocket supports the following commands:

    System commands  
    ---------------  
    Ping as boolean:  Returns True if the remote control socket is connected to CoolTerm.
    LastSocketError as integer:  Returns the last socket error

    Window/App commands
    -------------------
    NewWindow as integer:  Opens a new terminal window. Returns the ID of the new window.
    LoadSetting(FilePath as String) as integer:  Loads connection settings. Returns the ID of the opened window, or -1 if loading was not successful.
    SaveSetting(ID as integer, FilePath as String) as Boolean:  Saves the current connection settings. Returns True on success.
    WindowCount as integer:  Returns the number of currently open terminal windows.
    WindowID(index as integer) as integer:  Returns the ID of the window with the specified index.
    WindowIDfromName(WindowName as string) as integer:  Returns the ID of the window with the specified name, or -1 if the window doesn't exist.
    WindowName(index as integer) as string:  Returns the name of the window with the specified index.
    IndexOfWindowID(ID as integer) as integer:  Returns the index of the window with the specified ID.
    CloseWindow(ID as integer):  Closes the specified window.
    Quit:  Quits CoolTerm.
    CoolTermVersion as string:  Returns the CoolTerm version.
    ShowWindow(ID as integer):  Brings the specified window to the foreground.
    Print(ID as integer) as Boolean:  Prints the current contents of the specified window. Returns True on success.
    GetFrontmostWindow as integer: Returns the ID of the frontmost terminal window. Returns -1 if there are no open or visible windows.
    PauseDisplay(ID as integer, Value as boolean):  Pauses or unpauses the display of the specified window.

    Serial Port commands
    --------------------
    Connect(ID as integer) as Boolean:  Opens the serial port. Returns True on success.
    Disconnect(ID as integer):  Closes the serial port.
    IsConnected(ID as integer) as Boolean:  Returns true if the serial port is open.
    LastErrorCode(ID as Integer) as integer:  Returns the last serial port error code.

    Data exchange commands
    ----------------------
    Write(ID as integer, Data as string):  Writes the data to the serial port.
    WriteLine(ID as integer, Data as string):  Writes the data to the serial port and terminates it with the currently selected 'Enter Key Emulation'.
    WriteHex(ID as integer, Data as string):  Writes the Hex-formatted data to the serial port.
    BytesLeftToSend(ID as integer) as integer:  Returns the number of bytes left in the transmit buffer awaiting transmission.
    Poll(ID as integer):  Polls the serial port.
    Read(ID as integer, NumChar as integer) as string:  Reads the specified number of characters (bytes) from the receive buffer.
    ReadAll(ID as integer) as string:  Reads the entire contents of the receive buffer.
    ReadHex(ID as integer, NumChar as integer) as string:  Reads the specified number of characters (bytes) from the receive buffer, in Hex format.
    ReadAllHex(ID as integer) as string:  Reads the entire contents of the receive buffer, in Hex format.
    BytesAvailable(ID as integer) as integer:  Returns the number of bytes currently available in the receive buffer.
    LookAhead(ID as integer) as string:  Returns the contents of the receive buffer without removing it.
    LookAheadHex(ID as integer) as string:  Returns the contents of the receive buffer, in Hex format, without removing it.
    ClearBuffer(ID as integer):  Clears the contents of the receive buffer.

    Serial commands
    ---------------
    SendBreak(ID as integer):  Sends a break signal.
    FlushPort(ID as integer):  Flushes the serial port buffers.
    ResetPort(ID as integer):  Resets the serial port.
    GetDTR(ID as integer) as Boolean:  Returns the state of the DTR status line.
    SetDTR(ID as integer, Value as boolean):  Sets the state of the DTR status line.
    GetRTS(ID as integer) as Boolean:  Returns the state of the RTS status line.
    SetRTS(ID as Integer, Value as Boolean):  Sets the state of the RTS status line.
    GetCTS(ID as integer) as Boolean:  Returns the state of the CTS status line.
    GetDSR(ID as integer) as Boolean:  Returns the state of the DSR status line.
    GetDCD(ID as integer) as Boolean:  Returns the state of the DCD status line.
    GetRI(ID as integer) as Boolean:  Returns the state of the RI status line.
    SetBreak(ID as integer, Value as boolean):  Sets the state of the break signal.
    GetBreak(ID as integer) as boolean:  Gets the state of the break signal.

    Text data exchange
    ------------------
    SendTextFile(ID as integer, FilePath as string) as Boolean:  Sends the specified text file. Returns True on success.
    CaptureStart(ID as integer, FilePath as string) as Boolean:  Starts capture of received data to the specified file. Returns True on successful creation of the capture file.
    CapturePause(ID as integer):  Pauses the capture currently in progress.
    CaptureResume(ID as integer):  Resumes the currently paused capture.
    CaptureStop(ID as integer):  Stops the capture currently in progress and closes the capture file.

    Connection Setting Commands
    ---------------------------
    RescanSerialPorts:  Rescans the system for available serial ports.
    SerialPortCount as integer:  Returns the number of available serial ports.
    SerialPortName(SerialPortIndex as integer) as string:  Returns the name of the serial port with the specified index.
    GetCurrentSerialPort(ID as integer) as integer:  Returns the index of the currently selected serial ports.
    SetCurrentSerialPort(ID as integer, SerialPortIndex as integer) as Boolean:  Selects the serial port with the specified index. This can only be done while the port is closed.
    GetParameter(ID as integer, ParameterName as string) as string:  Gets the value of the parameter with the specified name.
    SetParameter(ID as integer, ParameterName as string, Value as string) as Boolean:  Sets the value of the parameter with the specified name.
    GetAllParameters(ID as integer) as string:  Returns a list of all parameters (one per line)
"""

import socket
import random
import time
import sys

class CoolTermSocket:
    
    def __init__(self, Host="127.0.0.1", Port=51413):
        # System commands  
        # ---------------  
        self.OP_PING = 0  # Ping as boolean
        self.OP_LAST_SOCKET_ERROR = 1  # LastSocketError as integer

        # Window/App commands
        # -------------------
        self.OP_NEW_WINDOW = 20  # NewWindow as integer
        self.OP_LOAD_SETTING = 21  # LoadSetting(FilePath as String) as integer
        self.OP_SAVE_SETTING = 22  # SaveSetting(ID as integer, FilePath as String) as Boolean
        self.OP_GET_WINDOW_COUNT = 23  # WindowCount as integer
        self.OP_GET_WINDOW_ID = 24  # WindowID(index as integer) as integer
        self.OP_GET_WINDOW_ID_FROM_NAME = 25  # WindowIDfromName(WindowName as string) as integer
        self.OP_GET_WINDOW_NAME = 26  # WindowName(index as integer) as string
        self.OP_INDEX_OF_WINDOW_ID = 27  # IndexOfWindowID(ID as integer) as integer
        self.OP_CLOSE_WINDOW = 28  # CloseWindow(ID as integer)
        self.OP_QUIT = 29  # Quit
        self.OP_VERSION = 30  # CoolTermVersion as string
        self.OP_SHOW_WINDOW = 31  # ShowWindow(ID as integer)
        self.OP_PRINT = 32  # Print(ID as integer) as Boolean
        self.OP_GET_FRONTMOST_WINDOW = 33  # GetFrontmostWindow as integer
        self.OP_PAUSE_DISPLAY = 34  # PauseDisplay(ID as integer, Value as boolean)

        # Serial Port commands
        # --------------------
        self.OP_CONNECT = 40  # Connect(ID as integer) as Boolean
        self.OP_DISCONNECT = 41  # Disconnect(ID as integer)
        self.OP_IS_CONNECTED = 42  # IsConnected(ID as integer) as Boolean
        self.OP_LAST_ERROR = 43  # LastErrorCode(ID as Integer) as integer

        # Data exchange commands
        # ----------------------
        self.OP_WRITE = 50  # Write(ID as integer, Data as string)
        self.OP_WRITE_LINE = 51  # WriteLine(ID as integer, Data as string)
        self.OP_WRITE_HEX = 52  # WriteHex(ID as integer, Data as string)
        self.OP_BYTES_LEFT_TO_SEND = 53  # BytesLeftToSend(ID as integer) as integer
        self.OP_POLL = 54  # Poll(ID as integer)
        self.OP_READ = 55  # Read(ID as integer, NumChar as integer) as string
        self.OP_READ_ALL = 56  # ReadAll(ID as integer) as string
        self.OP_READ_HEX = 57  # ReadHex(ID as integer, NumChar as integer) as string
        self.OP_READ_ALL_HEX = 58  # ReadAllHex(ID as integer) as string
        self.OP_BYTES_AVAILABLE = 59  # BytesAvailable(ID as integer) as integer
        self.OP_LOOK_AHEAD = 60  # LookAhead(ID as integer) as string
        self.OP_LOOK_AHEAD_HEX = 61  # LookAheadHex(ID as integer) as string
        self.OP_CLEAR_BUFFER = 62  # ClearBuffer(ID as integer)

        # Serial commands
        # ---------------
        self.OP_SEND_BREAK = 70  # SendBreak(ID as integer)
        self.OP_FLUSH_PORT = 71  # FlushPort(ID as integer)
        self.OP_RESET_PORT = 72  # ResetPort(ID as integer)
        self.OP_GET_DTR = 73  # GetDTR(ID as integer) as Boolean
        self.OP_SET_DTR = 74  # SetDTR(ID as integer, Value as boolean)
        self.OP_GET_RTS = 75  # GetRTS(ID as integer) as Boolean
        self.OP_SET_RTS = 76  # SetRTS(ID as Integer, Value as Boolean)
        self.OP_GET_CTS = 77  # GetCTS(ID as integer) as Boolean
        self.OP_GET_DSR = 78  # GetDSR(ID as integer) as Boolean
        self.OP_GET_DCD = 79  # GetDCD(ID as integer) as Boolean
        self.OP_GET_RI = 80  # GetRI(ID as integer) as Boolean
        # v1.3, added 2 line
        self.OP_SET_BREAK = 81  # SetBreak(ID as integer, Value as boolean)
        self.OP_GET_BREAK = 82  # GetBreak(ID as integer) as boolean

        # Text data exchange
        # ------------------
        self.OP_SEND_TEXTFILE = 90  # SendTextFile(ID as integer, FilePath as string) as Boolean
        self.OP_CAPTURE_START = 91  # CaptureStart(ID as integer, FilePath as string) as Boolean
        self.OP_CAPTURE_PAUSE = 92  # CapturePause(ID as integer)
        self.OP_CAPTURE_RESUME = 93  # CaptureResume(ID as integer)
        self.OP_CAPTURE_STOP = 94  # CaptureStop(ID as integer)

        # Connection Setting Commands
        # ---------------------------
        self.OP_RESCAN_SERIALPORTS = 100  # RescanSerialPorts
        self.OP_GET_SERIALPORT_COUNT = 101  # SerialPortCount as integer
        self.OP_GET_SERIALPORT_NAME = 102  # SerialPortName(SerialPortIndex as integer) as string
        self.OP_GET_CURRENT_SERIALPORT = 103  # GetCurrentSerialPort(ID as integer) as integer
        self.OP_SET_CURRENT_SERIALPORT = 104  # SetCurrentSerialPort(ID as integer, SerialPortIndex as integer) as Boolean
        self.OP_GET_PARAMETER = 110  # GetParameter(ID as integer, ParameterName as string) as string
        self.OP_SET_PARAMETER = 111  # SetParameter(ID as integer, ParameterName as string, Value as string) as Boolean
        self.OP_GET_ALL_PARAMETERS = 112  # GetAllParameters(ID as integer) as string

        # ACK codes
        # ---------
        self.ACK_OFFLINE = 251
        self.ACK_TIMEOUT = 252
        self.ACK_BAD_ARGUMENT = 253
        self.ACK_BAD_OPCODE = 254
        self.ACK_SUCCESS = 255


        # Connecting to CoolTerm
        self.Host = Host
        self.Port = Port
        self.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.skt.connect((self.Host,self.Port))
        except:
            print("ERROR: Could not connect to CoolTerm")
            #sys.exit()

    def __del__(self):
        self.skt.close()
        
    def __str__(self):
        return "{}('{}',{})".format(self.__class__.__name__, self.Host, self.Port)

    def Close(self):
        self.skt.close()

    def _SendPacket(self, Packet):
        # Send the packet
        self.skt.sendall(Packet)
        # Get the response
        data = self.skt.recv(65535)
        return data
        
    def _GetPacket(self, OP, ID=0, DATA=""):
        ID = ID.to_bytes(1, byteorder='little')
        PID = (random.randint(0,255)).to_bytes(1, byteorder='little')
        OP = OP.to_bytes(1, byteorder='little')
        # v1.4, changed 1 line
        #DATA = DATA.encode()
        DATA = DATA.encode(encoding='raw_unicode_escape')
        LEN = (len(DATA)).to_bytes(2, byteorder='little')
        Packet = b'\x1f' + LEN + PID + OP + ID + DATA
        return Packet

    def _isAck(self, Packet):
        if Packet[0] == 31: # 0x1f
            # valid packet
            if Packet[4] == self.ACK_SUCCESS:
                return True
            else:
                return False
        else:
            return False

    def _getData(self, Packet):
            LEN = Packet[1:3]
            LEN = int.from_bytes(LEN, byteorder='little')
            DATA = Packet[6:6+LEN]
            # v1.4, changed 1 line
            # v1.2, changed 1 line
            #return DATA.decode()
            #return DATA.decode(encoding='ascii')
            return DATA.decode(encoding='raw_unicode_escape')


    # ============================================================================

    # System commands  
    # ---------------  

    def Ping(self):
        Packet = self._GetPacket(self.OP_PING)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def LastSocketError(self):
        Packet = self._GetPacket(self.OP_LAST_SOCKET_ERROR)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False


    # Window/App commands
    # -------------------

    def NewWindow(self):
        Packet = self._GetPacket(self.OP_NEW_WINDOW)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def LoadSetting(self, FilePath):
        Packet = self._GetPacket(self.OP_LOAD_SETTING, DATA=FilePath)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False
       
    def SaveSetting(self, ID, FilePath):
        Packet = self._GetPacket(self.OP_SAVE_SETTING, ID, FilePath)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def GetWindowCount(self):
        Packet = self._GetPacket(self.OP_GET_WINDOW_COUNT)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def WindowCount(self): # 1.4, duplicate from above, for consistency with AppleScript and help text
    	return self.GetWindowCount()
    
    def GetWindowID(self, Index):
        Packet = self._GetPacket(self.OP_GET_WINDOW_ID, DATA=str(Index))
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def WindowID(self, Index): # 1.4, duplicate from above, for consistency with AppleScript and help text
    	return self.GetWindowID(Index)
    	
    def GetWindowIDfromName(self, WindowName):
        Packet = self._GetPacket(self.OP_GET_WINDOW_ID_FROM_NAME, DATA=WindowName)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def WindowIDfromName(self, WindowName): # 1.4, duplicate from above, for consistency with AppleScript and help text
    	return self.GetWindowIDfromName(WindowName)
    	
    def GetWindowName(self, Index):
        Packet = self._GetPacket(self.OP_GET_WINDOW_NAME, DATA=str(Index))
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False
            
    def WindowName(self, Index): # 1.4, duplicate from above, for consistency with AppleScript and help text
    	return self.GetWindowName(Index)

    def IndexOfWindowID(self, ID):
        Packet = self._GetPacket(self.OP_INDEX_OF_WINDOW_ID, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def CloseWindow(self, ID):
        Packet = self._GetPacket(self.OP_CLOSE_WINDOW, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def Quit(self):
        Packet = self._GetPacket(self.OP_QUIT)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def CoolTermVersion(self):
        Packet = self._GetPacket(self.OP_VERSION)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False
    
    def ShowWindow(self, ID):
        Packet = self._GetPacket(self.OP_SHOW_WINDOW, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)
    
    def Print(self, ID):
        Packet = self._GetPacket(self.OP_PRINT, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)
    
    def GetFrontmostWindow(self):
        Packet = self._GetPacket(self.OP_GET_FRONTMOST_WINDOW)
        Response = self._SendPacket(Packet)   
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def PauseDisplay(self, ID, Value):
        Packet = self._GetPacket(self.OP_PAUSE_DISPLAY, ID, str(Value))
        Response = self._SendPacket(Packet)
        return self._isAck(Response)


    # Serial Port commands
    # --------------------

    def Connect(self, ID):
        Packet = self._GetPacket(self.OP_CONNECT, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def Disconnect(self, ID):
        Packet = self._GetPacket(self.OP_DISCONNECT, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def IsConnected(self, ID):
        Packet = self._GetPacket(self.OP_IS_CONNECTED, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def LastError(self, ID):
        Packet = self._GetPacket(self.OP_LAST_ERROR, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False


    # Data exchange commands
    # ----------------------

    def Write(self, ID, Data):
        Packet = self._GetPacket(self.OP_WRITE, ID, Data)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def WriteLine(self, ID, Data):
        Packet = self._GetPacket(self.OP_WRITE_LINE, ID, Data)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def WriteHex(self, ID, HexData):
        Packet = self._GetPacket(self.OP_WRITE_HEX, ID, HexData)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def BytesLeftToSend(self, ID):
        Packet = self._GetPacket(self.OP_BYTES_LEFT_TO_SEND, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def Poll(self, ID):
        Packet = self._GetPacket(self.OP_POLL, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)


    def Read(self, ID, NumBytes):
        Packet = self._GetPacket(self.OP_READ, ID, str(NumBytes))
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def ReadAll(self, ID):
        Packet = self._GetPacket(self.OP_READ_ALL, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def ReadHex(self, ID, NumBytes):
        Packet = self._GetPacket(self.OP_READ_HEX, ID, str(NumBytes))
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def ReadAllHex(self, ID):
        Packet = self._GetPacket(self.OP_READ_ALL_HEX, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def BytesAvailable(self, ID):
        Packet = self._GetPacket(self.OP_BYTES_AVAILABLE, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def LookAhead(self, ID):
        Packet = self._GetPacket(self.OP_LOOK_AHEAD, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def LookAheadHex(self, ID):
        Packet = self._GetPacket(self.OP_LOOK_AHEAD_HEX, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def ClearBuffer(self, ID):
        Packet = self._GetPacket(self.OP_CLEAR_BUFFER, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)


    # Serial commands
    # ---------------

    def SendBreak(self, ID):
        Packet = self._GetPacket(self.OP_SEND_BREAK, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def FlushPort(self, ID):
        Packet = self._GetPacket(self.OP_FLUSH_PORT, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def ResetPort(self, ID):
        Packet = self._GetPacket(self.OP_RESET_PORT, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def GetDTR(self, ID):
        Packet = self._GetPacket(self.OP_GET_DTR, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def SetDTR(self, ID, State):
        Packet = self._GetPacket(self.OP_SET_DTR, ID, str(State))
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def GetRTS(self, ID):
        Packet = self._GetPacket(self.OP_GET_RTS, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def SetRTS(self, ID, State):
        Packet = self._GetPacket(self.OP_SET_RTS, ID, str(State))
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def GetCTS(self, ID):
        Packet = self._GetPacket(self.OP_GET_CTS, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def GetDSR(self, ID):
        Packet = self._GetPacket(self.OP_GET_DSR, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def GetDCD(self, ID):
        Packet = self._GetPacket(self.OP_GET_DCD, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def GetRI(self, ID):
        Packet = self._GetPacket(self.OP_GET_RI, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False
    
    def SetBreak(self, ID, State): # v1.3, added method
        Packet = self._GetPacket(self.OP_SET_BREAK, ID, str(State))
        Response = self._SendPacket(Packet)
        return self._isAck(Response)
        
    def GetBreak(self, ID): # v1.3, added method
        Packet = self._GetPacket(self.OP_GET_BREAK, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False


    # Text data exchange
    # ------------------

    def SendTextFile(self, ID, FilePath):
        Packet = self._GetPacket(self.OP_SEND_TEXTFILE, ID, FilePath)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def CaptureStart(self, ID, FilePath):
        Packet = self._GetPacket(self.OP_CAPTURE_START, ID, FilePath)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def CapturePause(self, ID):
        Packet = self._GetPacket(self.OP_CAPTURE_PAUSE, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def CaptureResume(self, ID):
        Packet = self._GetPacket(self.OP_CAPTURE_RESUME, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def CaptureStop(self, ID):
        Packet = self._GetPacket(self.OP_CAPTURE_STOP, ID)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)


    # Connection Setting Commands
    # ---------------------------

    def RescanSerialPorts(self):
        Packet = self._GetPacket(self.OP_RESCAN_SERIALPORTS)
        Response = self._SendPacket(Packet)
        return self._isAck(Response)

    def GetSerialPortCount(self):
        Packet = self._GetPacket(self.OP_GET_SERIALPORT_COUNT)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def GetSerialPortName(self, SerialPortIndex):
        Packet = self._GetPacket(self.OP_GET_SERIALPORT_NAME, DATA=str(SerialPortIndex))
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def GetCurrentSerialPort(self, ID):
        Packet = self._GetPacket(self.OP_GET_CURRENT_SERIALPORT, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return int(self._getData(Response))
        else:
            return False

    def SetCurrentSerialPort(self, ID, SerialPortIndex):
        Packet = self._GetPacket(self.OP_SET_CURRENT_SERIALPORT, ID, str(SerialPortIndex))
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def GetParameter(self, ID, ParameterName):
        Packet = self._GetPacket(self.OP_GET_PARAMETER, ID, ParameterName)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

    def SetParameter(self, ID, ParameterName, Value):
        Packet = self._GetPacket(self.OP_SET_PARAMETER, ID, ParameterName + '\x00' + str(Value))
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            if self._getData(Response) == "True":
                return True
            else:
                return False
        else:
            return False

    def GetAllParameters(self, ID):
        Packet = self._GetPacket(self.OP_GET_ALL_PARAMETERS, ID)
        Response = self._SendPacket(Packet)
        if self._isAck(Response):
            return self._getData(Response)
        else:
            return False

