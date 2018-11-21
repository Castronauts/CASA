# tcpcom.py
# AP

'''
 This software is part of the TCPCom library.
 It is Open Source Free Software, so you may
 - run the code for any purpose
 - study how the code works and adapt it to your needs
 - integrate all or parts of the code in your own programs
 - redistribute copies of the code
 - improve the code and release your improvements to the public
 However the use of the code is entirely your responsibility.
 '''

from threading import Thread
import thread
import socket
import time
import sys

TCPCOM_VERSION = "1.15 - Feb. 15, 2016"

# ================================== Server ================================
# ---------------------- class TCPServer ------------------------
class TCPServer(Thread):
    '''
    Class that represents a TCP socket based server.
    '''
    isVerbose = False
    PORT_IN_USE = "PORT_IN_USE"
    CONNECTED = "CONNECTED"
    LISTENING = "LISTENING"
    TERMINATED = "TERMINATED"
    MESSAGE = "MESSAGE"

    def __init__(self, port, stateChanged, isVerbose = False):
        '''
        Creates a TCP socket server that listens on TCP port
        for a connecting client. The server runs in its own thread, so the
        constructor returns immediately. State changes invoke the callback
        onStateChanged().
        @param port: the IP port where to listen (0..65535)
        @param stateChange: the callback function to register
        @param isVerbose: if true, debug messages are written to System.out, default: False
        '''
        Thread.__init__(self)
        self.port = port
        self.stateChanged = stateChanged
        TCPServer.isVerbose = isVerbose
        self.isClientConnected = False
        self.terminateServer = False
        self.isServerRunning = False
        self.start()

    def run(self):
        TCPServer.debug("TCPServer thread started")
        HOSTNAME = "" # Symbolic name meaning all available interfaces
        self.conn = None
        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # close port when process exits
        TCPServer.debug("Socket created")
        try:
            self.serverSocket.bind((HOSTNAME, self.port))
        except socket.error as msg:
            print "Fatal error while creating TCPServer: Bind failed.", msg[0], msg[1]
            sys.exit()
        try:    
            self.serverSocket.listen(10)
        except:
            print "Fatal error while creating TCPServer: Port", self.port, "already in use"
            try:
                self.stateChanged(TCPServer.PORT_IN_USE, str(self.port))
            except Exception, e:
               print "Caught exception in TCPServer.PORT_IN_USE:", e
            sys.exit()

        try:
            self.stateChanged(TCPServer.LISTENING, str(self.port))
        except Exception, e:
            print "Caught exception in TCPServer.LISTENING:", e

        self.isServerRunning = True
                
        while True:
            TCPServer.debug("Calling blocking accept()...")
            conn, self.addr = self.serverSocket.accept()
            if self.terminateServer:
                self.conn = conn
                break
            if self.isClientConnected:
                TCPServer.debug("Returning form blocking accept(). Client refused")
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except:
                    pass
                conn.close()
                continue
            self.conn = conn
            self.isClientConnected = True
            self.socketHandler = ServerHandler(self)
            self.socketHandler.setDaemon(True)  # necessary to terminate thread at program termination
            self.socketHandler.start()
            try: 
                self.stateChanged(TCPServer.CONNECTED, self.addr[0])
            except Exception, e:
                print "Caught exception in TCPServer.CONNECTED:", e
        self.conn.close()
        self.serverSocket.close()
        self.isClientConnected = False
        try:
            self.stateChanged(TCPServer.TERMINATED, "")
        except Exception, e:
            print "Caught exception in TCPServer.TERMINATED:", e
        self.isServerRunning = False
        TCPServer.debug("TCPServer thread terminated")

    def terminate(self):
        '''
        Closes the connection and terminates the server thread.
        Releases the IP port.
        '''
        TCPServer.debug("Calling terminate()")
        if not self.isServerRunning:
            TCPServer.debug("Server not running")
            return
        self.terminateServer = True
        TCPServer.debug("Disconnect by a dummy connection...")
        if self.conn != None:
            self.conn.close()
            self.isClientConnected = False
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', self.port))  # dummy connection to get out of accept()
        
    def disconnect(self):
        '''
        Closes the connection with the client and enters
        the LISTENING state
        '''
        TCPServer.debug("Calling Server.disconnect()")
        if self.isClientConnected:
            self.isClientConnected = False
            try:
                self.stateChanged(TCPServer.LISTENING, str(self.port))
            except Exception, e:
                print "Caught exception in TCPServer.LISTENING:", e
            TCPServer.debug("Shutdown socket now")
            try:
                self.conn.shutdown(socket.SHUT_RDWR)
            except:
                pass
            self.conn.close()

    def sendMessage(self, msg):
        '''
        Sends the information msg to the client (as String, the character \0 (ASCII 0) serves as end of
        string indicator, it is transparently added and removed)
        @param msg: the message to send
        '''
        TCPServer.debug("sendMessage() with msg: " + msg)
        if not self.isClientConnected:
            TCPServer.debug("Not connected")
            return
        try:
            self.conn.sendall(msg + "\0")    
        except:
            TCPClient.debug("Exception in sendMessage()")

    def isConnected(self):
        '''
        Returns True, if a client is connected to the server.
        @return: True, if the communication link is established
        '''
        return self.isClientConnected
    
    def isTerminated(self):
        '''
        Returns True, if the server is in TERMINATED state.
        @return: True, if the server thread is terminated
        '''
        return self.terminateServer

    @staticmethod
    def debug(msg):
        if TCPServer.isVerbose:
            print "   TCPServer-> " + msg
 
    @staticmethod
    def getVersion():
        '''
        Returns the library version.
        @return: the current version of the library
        '''
        return TCPCOM_VERSION
   
# ---------------------- class ServerHandler ------------------------
class ServerHandler(Thread):
    def __init__(self, server):
        Thread.__init__(self)
        self.server = server

    def run(self):
        TCPServer.debug("ServerHandler started")
        bufSize = 4096
        try:
            while True:
                data = ""
                reply = ""
                isRunning = True
                while not reply[-1:] == "\0":
                    TCPServer.debug("Calling blocking conn.recv()")
                    reply = self.server.conn.recv(bufSize)
                    if reply == None or len(reply) == 0: # Client disconnected
                        TCPServer.debug("conn.recv() returned None")
                        isRunning = False
                        break
                    data += reply
                if not isRunning:
                    break
                TCPServer.debug("Received msg: " + data + " len: " + str(len(data)))
                junk = data.split("\0")  # more than 1 message may be received if
                                         # transfer is fast. data: xxxx\0yyyyy\0zzz\0
                for i in range(len(junk) - 1):
                    try:
                        self.server.stateChanged(TCPServer.MESSAGE, junk[i])
                    except Exception, e:
                        print "Caught exception in TCPServer.MESSAGE:", e
        except:  # May happen if client peer is resetted
            TCPServer.debug("Exception from blocking conn.recv(), Msg: " + str(sys.exc_info()[0]) + \
              " at line # " +  str(sys.exc_info()[-1].tb_lineno))

        self.server.disconnect()
        TCPServer.debug("ServerHandler terminated")


# ================================== Client ================================
# -------------------------------- class TCPClient --------------------------
class TCPClient():
    '''
    Class that represents a TCP socket based client.
    '''
    isVerbose = False
    CONNECTING = "CONNECTING"
    SERVER_OCCUPIED = "SERVER_OCCUPIED"
    CONNECTION_FAILED = "CONNECTION_FAILED"
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    MESSAGE = "MESSAGE"

    def __init__(self, ipAddress, port, stateChanged, isVerbose = False):
        '''
        Creates a TCP socket client prepared for a connection with a
        TCPServer at given address and port.
        @param host: the IP address of the host
        @param port: the IP port where to listen (0..65535)
        @param stateChanged: the callback function to register
        @param isVerbose: if true, debug messages are written to System.out
        '''
        self.isClientConnected = False
        self.isClientConnecting = False
        self.ipAddress = ipAddress
        self.port = port
        self.stateChanged = stateChanged
        self.checkRefused = False
        self.isRefused = False

        TCPClient.isVerbose = isVerbose
                  
    def sendMessage(self, msg, responseTime = 0):
        '''
        Sends the information msg to the server (as String, the character \0
        (ASCII 0) serves as end of string indicator, it is transparently added
        and removed).  For responseTime > 0 the method blocks and waits
        for maximum responseTime seconds for a server reply.
        @param msg: the message to send
        @param responseTime: the maximum time to wait for a server reply (in s)
        @return: the message or null, if a timeout occured
        '''
        TCPClient.debug("sendMessage() with msg = " + msg)
        if not self.isClientConnected:
            TCPClient.debug("sendMessage(): Connection closed.")
            return None
        reply = None
        try:
            msg += "\0";  # Append \0
            rc = self.sock.sendall(msg)
            if responseTime > 0:
                reply = self._waitForReply(responseTime)  # Blocking
        except:
            TCPClient.debug("Exception in sendMessage()")
            self.disconnect()
    
        return reply
    
    def _waitForReply(self, responseTime):
        TCPClient.debug("Calling _waitForReply()")
        self.receiverResponse = None
        startTime = time.time()
        while self.isClientConnected and self.receiverResponse == None and time.time() - startTime < responseTime:
            time.sleep(0.01)
        if self.receiverResponse == None:
            TCPClient.debug("Timeout while waiting for reply")
        else:    
            TCPClient.debug("Response = " + self.receiverResponse + " time elapsed: " + str(int(1000 * (time.time() - startTime))) + " ms")
        return self.receiverResponse

    def connect(self, timeout = 0):
        '''
        Creates a connection to the server (blocking until timeout).
        @param timeout: the maximum time (in s) for the connection trial  (0: for default timeout)
        @return: True, if the connection is established; False, if the server
        is not available or occupied
        '''
        if timeout == 0:
            timeout = None
        try:
            self.stateChanged(TCPClient.CONNECTING, self.ipAddress + ":" + str(self.port))
        except Exception, e:
            print "Caught exception in TCPClient.CONNECTING:", e
        try:
            self.isClientConnecting = True
            host = (self.ipAddress, self.port)
            if self.ipAddress == "localhost" or self.ipAddress == "127.0.0.1":
                timeout = None  # do not use timeout for local host, to avoid error message "java.net..."
            self.sock = socket.create_connection(host, timeout)
            self.sock.settimeout(None)
            self.isClientConnecting = False
            self.isClientConnected = True
        except:
            self.isClientConnecting = False
            try:
                self.stateChanged(TCPClient.CONNECTION_FAILED, self.ipAddress + ":" + str(self.port))
            except Exception, e:
                print "Caught exception in TCPClient.CONNECTION_FAILED:", e
            TCPClient.debug("Connection failed.")
            return False
        ClientHandler(self)

        # Check if connection is refused
        self.checkRefused = True
        self.isRefused = False
        startTime = time.time()
        while time.time() - startTime < 2 and not self.isRefused:
            time.sleep(0.001)
        if self.isRefused:
            TCPClient.debug("Connection refused")
            try:
                self.stateChanged(TCPClient.SERVER_OCCUPIED, self.ipAddress + ":" + str(self.port))
            except Exception, e:
                print "Caught exception in TCPClient.SERVER_OCCUPIED:", e
            return False

        try:
            self.stateChanged(TCPClient.CONNECTED, self.ipAddress + ":" + str(self.port))
        except Exception, e:
            print "Caught exception in TCPClient.CONNECTED:", e
        TCPClient.debug("Successfully connected")
        return True

    def disconnect(self):
        '''
        Closes the connection with the server.
        '''
        TCPClient.debug("Client.disconnect()")
        if not self.isClientConnected:
            TCPClient.debug("Connection already closed")
            return
        self.isClientConnected = False
        TCPClient.debug("Closing socket")
        try: # catch Exception "transport endpoint is not connected"
            self.sock.shutdown(socket.SHUT_RDWR)
        except:
            pass
        self.sock.close()

    def isConnecting(self):
        '''
        Returns True during a connection trial.
        @return: True, while the client tries to connect
        '''
        return self.isClientConnecting

    def isConnected(self):
        '''
        Returns True of client is connnected to the server.
        @return: True, if the connection is established
        '''
        return self.isClientConnected
    
    @staticmethod
    def debug(msg):
        if TCPClient.isVerbose:
            print "   TCPClient-> " + msg

    @staticmethod
    def getVersion():
        '''
        Returns the library version.
        @return: the current version of the library
        '''
        return TCPCOM_VERSION

# -------------------------------- class ClientHandler ---------------------------
class ClientHandler(Thread):
    def __init__(self, client):
        Thread.__init__(self)
        self.client = client
        self.start()
                
    def run(self):
        TCPClient.debug("ClientHandler thread started")
        while True:
            try:
                junk = self.readResponse().split("\0")
                # more than 1 message may be received 
                # if transfer is fast. data: xxxx\0yyyyy\0zzz\0
                for i in range(len(junk) - 1):
                    try:
                        self.client.stateChanged(TCPClient.MESSAGE, junk[i])
                    except Exception, e:
                        print "Caught exception in TCPClient.MESSAGE:", e
            except:    
                TCPClient.debug("Exception in readResponse() Msg: " + str(sys.exc_info()[0]) + \
                  " at line # " +  str(sys.exc_info()[-1].tb_lineno))
                if self.client.checkRefused:
                    self.client.isRefused = True
                break
        try:
            self.client.stateChanged(TCPClient.DISCONNECTED, "")
        except Exception, e:
            print "Caught exception in TCPClient.DISCONNECTED:", e
        TCPClient.debug("ClientHandler thread terminated")

    def readResponse(self):
        TCPClient.debug("Calling readResponse")
        bufSize = 4096
        data = ""
        while not data[-1:]  ==  "\0":
            try:
                reply = self.client.sock.recv(bufSize)  # blocking
                if len(reply) == 0:
                    TCPClient.debug("recv returns null length")
                    raise Exception("recv returns null length")
            except:
                TCPClient.debug("Exception from blocking conn.recv(), Msg: " + str(sys.exc_info()[0]) + \
                  " at line # " +  str(sys.exc_info()[-1].tb_lineno))
                raise Exception("Exception from blocking sock.recv()")
            data += reply
            self.receiverResponse = data[:-1]
        return data


