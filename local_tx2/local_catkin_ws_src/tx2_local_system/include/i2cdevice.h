
#ifndef I2C_H_
#define I2C_H_

#define BBB_I2C_0 "/dev/i2c-6"
#define BBB_I2C_1 "/dev/i2c-6"
#define BBB_I2C_2 "/dev/i2c-6"
namespace exploringBB {

/**
 * @class I2CDevice
 * @brief Generic I2C Device class that can be used to connect to any type of I2C device and read or write to its registers
 */
class I2CDevice{
private:
	unsigned int bus;
	unsigned int device;
	int file;
public:
	I2CDevice(unsigned int bus, unsigned int device);
	virtual int open();
	virtual int write(unsigned char value);
	virtual unsigned char readRegister(unsigned int registerAddress);
	virtual unsigned char* readRegisters(unsigned int number, unsigned int fromAddress=0);
	unsigned char* readLine(unsigned int number);
	unsigned char readbyte(void);
	virtual int writeRegisterblock(unsigned int registerAddress, unsigned char *value, int buffersize);
	virtual int writeRegister(unsigned int registerAddress, unsigned char value);
	virtual int writeblock(unsigned char *value, int buffersize);
	virtual void debugDumpRegisters(unsigned int number = 0xff);
	virtual void close();
	virtual ~I2CDevice();
};

} /* namespace exploringBB */

#endif /* I2C_H_ */
