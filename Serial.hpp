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
    Serial();
    ~Serial();  
    /**
     * Connect to a serial interface like /dev/ttyUSB0, /dev/tty1, or COM1, COM2 on Windows.
     * @param serial interface (/dev/ttyUSB0, COM1)
     * @param Baud rate of the communication (9600, 19200,...)
     * @param struct termios with options setted.
     * @return True/False on connect
    */
    bool connect(const char *device, int baudRate, struct termios * opt);
   
    /**
     * Closes the serial file descriptor.
    */
    void disconnect(void); 
    
    /**
     * Send message throgh the serial interface.
     * @param buffer containing the message to send
     * @param message size
     * @return True: Message sended successfully. False: Message not sended, error.
    */
    bool sendArray(unsigned char *buffer, int len);
    /**
     * Read an array from the serial interface.
     * @param buffer where the message will be stored
     * @param buffer max size
     * @return size of the message readed, -1: error.
    */
    int getArray (unsigned char *buffer, int len);

private:
    int fileDescriptor;
    bool connected;
    long getBaudRateFlag(int);
    struct termios options;
 
};

#endif /* SERIAL_HPP */

