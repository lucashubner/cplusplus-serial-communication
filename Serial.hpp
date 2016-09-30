/* 
 * File:   Serial.hpp
 * @author: Lucas Guilherme Hübner
 *
 * Created on 14 de Setembro de 2016, 08:12
 */

#ifndef SERIAL_HPP
#define SERIAL_HPP
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <iostream>

#include <cstdio>
#include <exception>
#include <cstring>
#include <sstream>

#include <sys/ioctl.h>

class SerialException : public std::exception{
    public:
        SerialException(const std::string m="Exception!"):message(m){};
        ~SerialException(void) throw(){};
        const char * what() const throw() {return message.c_str();}
    private:
        std::string message;
};

class Serial{
public:
    Serial(int type = 0);
    ~Serial();  
    
    void setSerialType(int);
    /**
     * Connect to a serial interface like /dev/ttyUSB0, /dev/tty1, or COM1, COM2 on Windows.
     * @param serial interface (/dev/ttyUSB0, COM1)
     * @param Baud rate of the communication (9600, 19200,...)
     * @param struct termios with options setted.
     * @return True/False on connect
     * @throws SerialException
    */
    bool connect(const char *device, int baudRate, struct termios * opt = NULL);
    
    /**
     * Connect to a serial interface with options allrealy setted.
     * Requires to be connected once to use configured options or set
     * the flags manually.
     * @return True/False on connect
     * @throws SerialException
    */
    bool connect();
   
    /**
     * Closes the serial file descriptor.
    */
    void disconnect(void); 
    
    /**
     * Send message throgh the serial interface.
     * @param buffer containing the message to send
     * @param message size
     * @return size of the message sended.
     * @throws SerialException
    */
    int sendArray(unsigned char *buffer, int len);
    
    /**
     * Read an array from the serial interface.
     * @param buffer where the message will be stored
     * @param buffer max size
     * @return size of the message readed, -1: error.
     * @throws SerialException
    */
    int getArray (unsigned char *buffer, int len);

    /**
     * Configure default flags for communication
    */
    void configure();
    
    /**
     * Configure flag for Hardware Flow Control.
     * @param Boolean Enable(true)/Disable(false).
    */
    void setHardwareFlowControl(bool enable);
    
    /**
     * Configure flag for Software Flow Control.
     * @param Boolean Enable(true)/Disable(false).
    */
    void setSoftwareFlowControl(bool enable);
    
    /**
     * Configure flag for Raw Output.
     * @param Boolean Enable(true)/Disable(false).
    */
    void setRawOutput(bool enable);
    
    /**
     * This option causes the read funcion to return 0 if there's no characters
     * available on the serial port. If it's disabled, the read funcion will
     * return 0 when theres no character waiting.
     * @param Boolean Enable(true)/Disable(false).
    */
    void setBlockUntilRead(bool enable);
    
    /**
     * Set the device path, /dev/ttyUSB0, /dev/tty1, or COM1, COM2 on Windows.
     * @param String with serial interface (/dev/ttyUSB0, COM1).
    */
    void setDevice(std::string device);
        
    /**
     * Verify if the flag connected is setted.
     * @return Boolean value for connected flag.
    */
    bool isConnected();
    
private:
    // Private Functions
    long getBaudRateFlag(int);
    
    // Private Attributes
    bool rawOutput;
    bool blockUntilRead;
    bool softwareFlowControl;
    bool hardwareFlowControl;
    
    bool flagsSetted;
    bool connectedOnce;
    bool connected;
    
    bool deviceSetted;
    std::string device;
    
    int fileDescriptor;

    struct termios options;

    int serialType;
};

#endif /* SERIAL_HPP */

