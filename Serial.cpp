/* 
 * File:   Serial.cpp
 * @author: Lucas Guilherme Hübner
 * 
 * Created on 14 de Setembro de 2016, 08:12
 */

#include "Serial.hpp"

Serial::Serial() {
}
 //---------------------------------------------------------------------//
Serial::~Serial() {
}
//---------------------------------------------------------------------//
bool Serial::connect(const char * device, int baudRate, struct termios * opt /*= NULL*/) {
    this->fileDescriptor = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
            //                            |        |          |
            //                            |        |          +--- This program doesn't care what state the DCD signal line is in.
            //                            |        +--- This program doesn't want to be the "controlling terminal" for that port.
            //                            +--- Read and Write
    if(this->fileDescriptor == -1){
        throw SerialException("Can't open the port");
        return false;
    }
    
    fcntl(this->fileDescriptor, F_SETFL, FNDELAY);
    
    // Get the current flags from port
    tcgetattr(this->fileDescriptor, &this->options);
    
    // If flags not setted, assume default
    if(opt == NULL){
        this->options.c_cflag &= ~CSIZE; /* Mask the character size bits */
        this->options.c_cflag |= CS8;    /* Select 8 data bits */

        // Enable the receiver and set local mode...
        this->options.c_cflag |= (CLOCAL | CREAD);

        // Set parity - No Parity (8N1)
        this->options.c_cflag &= ~PARENB;
        this->options.c_cflag &= ~CSTOPB;
        this->options.c_cflag &= ~CSIZE;
        this->options.c_cflag |= CS8;
  
        // Enable Raw Input
        this->options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        // Disable Software Flow control

        this->options.c_iflag &= ~(IXON | IXOFF | IXANY);
 
        // Chose raw (not processed) output

        this->options.c_oflag &= ~OPOST;
    }
    else{
        this->options = *opt;
    }
    
    // Set the baudrate
    cfsetispeed(&this->options, this->getBaudRateFlag(baudRate));
    cfsetospeed(&this->options, this->getBaudRateFlag(baudRate));

    tcsetattr(this->fileDescriptor, TCSANOW, &options);
    fcntl(this->fileDescriptor, F_SETFL, FNDELAY);
    
    this->connected = true;
    return true;
}
//---------------------------------------------------------------------//
void Serial::disconnect() {
    close(this->fileDescriptor);
    this->connected = false;
}
//---------------------------------------------------------------------//
int Serial::getArray(unsigned char* buffer, int len) {
    if(!this->connected){
        throw SerialException("Not connected, please use connect(device, baudrate) to connect");
        return -1;
    }
    return read(this->fileDescriptor,buffer,len);
}
//---------------------------------------------------------------------//
bool Serial::sendArray(unsigned char* buffer, int len) {
    if(write(this->fileDescriptor, buffer, len)){
       return true; 
    }
    std::ostringstream oss;
    oss << "Error writing " << len << " bytes to serial";
    throw SerialException(oss.str());
    return false;
}
//---------------------------------------------------------------------//

long Serial::getBaudRateFlag(int baud){
    switch(baud){
        case 0:      return B0;
        case 50:     return B50;
        case 75:     return B75;
        case 110:    return B110;
        case 134:    return B134;	
        case 150:    return B150;	
        case 200:    return B200;	
        case 300:    return B300;	
        case 600:    return B600;	
        case 1200:   return B1200;
        case 1800:   return B1800;	
        case 2400:   return B2400;	
        case 4800:   return B4800;	
        case 9600:   return B9600;	
        case 19200:  return B19200;	
        case 38400:  return B38400;	
        case 57600:  return B57600;	
        case 115200: return B115200;
        default:     return B0;
    }
    
}

