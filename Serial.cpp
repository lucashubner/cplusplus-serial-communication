/*
 * File:   Serial.cpp
 * @author: Lucas Guilherme Hübner
 *
 * Created on 14 de Setembro de 2016, 08:12
 */

#include "../Headers/Serial.hpp"

Serial::Serial(int type /* = 0*/) {
	this->fileDescriptor = 0;
    this->serialType = type;
    this->connected = false;
    this->connectedOnce = false;
}
//---------------------------------------------------------------------//
void Serial::setSerialType(int type){
    this->serialType = type;
}
 //---------------------------------------------------------------------//
Serial::~Serial() {
    connected? close(this->fileDescriptor): 0;
}
//---------------------------------------------------------------------//
bool Serial::connect(const char * device, int baudRate, struct termios * opt /*= NULL*/) {
    this->device = device;
    this->fileDescriptor = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
            //                            |        |          |
            //                            |        |          +--- This program doesn't care what state the DCD signal line is in.
            //                            |        +--- This program doesn't want to be the "controlling terminal" for that port.
            //                            +--- Read and Write
    if(this->fileDescriptor == -1){
        throw SerialException("Can't open the port");
        return false;
    }

    // Get the current flags from port
    tcgetattr(this->fileDescriptor, &this->options);

    // Default == false
    setBlockUntilRead(true);

    if(opt == NULL)
        configure();
    else
        this->options = *opt;

    // Set the baudrate
    cfsetispeed(&this->options, this->getBaudRateFlag(baudRate));
    cfsetospeed(&this->options, this->getBaudRateFlag(baudRate));


    // Set the file descriptor options
    if(tcsetattr(this->fileDescriptor, TCSANOW, &options) <0){
        throw SerialException("Error setting the config flags.");
    }

    this->connectedOnce = true;
    this->connected = true;
    return true;
}
//---------------------------------------------------------------------//
bool Serial::connect() {
    if(!connectedOnce && (this->device.size() > 0)){
        throw SerialException("Flags not setted! Cant connect to device");
    }

    this->fileDescriptor = open(this->device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
                        //                            |        |          |
                        //                            |        |          +--- This program doesn't care what state the DCD signal line is in.
                        //                            |        +--- This program doesn't want to be the "controlling terminal" for that port.
                        //                            +--- Read and Write
    if(this->fileDescriptor == -1){
        throw SerialException("Can't open the port");
    }

    // Set the file descriptor options
    if(tcsetattr(this->fileDescriptor, TCSANOW, &options) <0){
        throw SerialException("Error setting the config flags.");
    }

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
    }

    return read(this->fileDescriptor,buffer,len);
}
//---------------------------------------------------------------------//
int Serial::sendArray(unsigned char* buffer, int len) {
    if(!this->connected){
        throw SerialException("Not connected, please use connect(device, baudrate) to connect");
    }

    int n = write(this->fileDescriptor, buffer, len);
    if(n > 0){
       return n;
    }
    std::ostringstream oss;
    oss << "Error writing " << len << " bytes to serial";
    throw SerialException(oss.str());
}
//---------------------------------------------------------------------//

void Serial::configure(){
	// Clear the flags from options
	this->options.c_cflag = 0;
	this->options.c_iflag = 0;

	this->options.c_ispeed = 0;
	this->options.c_ospeed = 0;

	this->options.c_lflag = 0;
	this->options.c_line = 0;
	this->options.c_oflag = 0;

    
// c_cflag
    // Enable the receiver and set local mode...
    this->options.c_cflag |= (CLOCAL | CREAD);

    // Set parity - No Parity (8N1)
    this->options.c_cflag &= ~PARENB;
    this->options.c_cflag &= ~CSTOPB;
    this->options.c_cflag &= ~CSIZE;
    this->options.c_cflag |= CS8;

    // Disable hardware flow control
    this->options.c_cflag &= ~CRTSCTS;

    // no signaling chars, no echo, no canonical processing
    this->options.c_lflag = 0;

// c_cc
    // read doesn't block
    this->options.c_cc[VMIN] = 1;
    // 0.5 seconds read timeout
    this->options.c_cc[VTIME] = 15;
    
//c_iflag
    // Disable Software flow control
    this->options.c_iflag &=  ~(IXON | IXOFF | IXANY);
    
    // Ignore BREAK condition on Input, Ignore Framing and Parity Errors·
    this->options.c_iflag |= IGNPAR | IGNBRK;

//c_oflag

    // Enable Raw Input
    this->options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Chose raw (not processed) output
    this->options.c_oflag &= ~OPOST;


}
//---------------------------------------------------------------------//
void Serial::setHardwareFlowControl(bool enable){
    this->options.c_cflag = enable? this->options.c_cflag | CRTSCTS :
                                    this->options.c_cflag & ~CRTSCTS;
}
//---------------------------------------------------------------------//
void Serial::setSoftwareFlowControl(bool enable){
    this->options.c_iflag = enable? this->options.c_iflag |  (IXON | IXOFF | IXANY):
                                    this->options.c_iflag & ~(IXON | IXOFF | IXANY);
}
//---------------------------------------------------------------------//
void Serial::setRawOutput(bool enable){
    this->options.c_oflag = enable? this->options.c_oflag |  OPOST
                                  : this->options.c_oflag & ~OPOST;
}
//---------------------------------------------------------------------//
void Serial::setBlockUntilRead(bool enable){
    this->fileDescriptor != -1? 0: throw ("File Descriptor not Setted, please connect once");
    enable? fcntl(this->fileDescriptor, F_SETFL, FNDELAY):
            fcntl(this->fileDescriptor, F_SETFL, 0      );

}
//---------------------------------------------------------------------//
void Serial::setDevice(std::string device) {
    this->device = device;
}
//---------------------------------------------------------------------//
bool Serial::isConnected() {
    return connected;
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
//---------------------------------------------------------------------//
