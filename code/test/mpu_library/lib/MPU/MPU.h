#include <Arduino.h>

enum mpu_t
{
        MPU6050
} typedef mpu_t;

#define MPU6050_ADDR 0x68

class MPU
{
public:
    MPU(mpu_t mpu) : _mpu(mpu), _device_addr(MPU6050_ADDR) {};

    void initialize();
    void setAddress(uint8_t addr);

private:
    mpu_t _mpu;
    uint8_t _device_addr;
};
