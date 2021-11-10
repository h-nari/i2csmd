#include "i2csmd.h"

#define PHASE_MAX 8
static const uint8_t drv_pat[PHASE_MAX] = {0x01, 0x03, 0x02, 0x06,
                                           0x04, 0x0c, 0x08, 0x09};

SteppingMotor::SteppingMotor() { init(); }

void SteppingMotor::init(void) {
  _bits = 0;
  _phase = 0;
  _bRev = false;
  _bRunning = false;
  _bForever = false;
  _bOffed = true;
  _stepPerRot = 64 * 8 * PHASE_MAX;
  _rpm = 10.0f;
  _curPos = _targetPos = 0;
}

void SteppingMotor::update(void) {
  uint32_t tNow = micros();
  if (_bRunning) {
    if (_tUpdate - tNow >= 0x8000000) {
      if (_bRev) {
        _curPos--;
        if (--_phase == 255) _phase = PHASE_MAX - 1;
      } else {
        _curPos++;
        if (++_phase >= PHASE_MAX) _phase = 0;
      }
      drive(_phase);
      if ((!_bForever && _curPos == _targetPos) || _tPeriod == 0) {
        _bRunning = 0;
        _tStop = tNow;
      } else if (_tPeriod > 0)
        _tUpdate += _tPeriod;
    }
  } else if (tNow - _tStop > 1000000 && !_bOffed)
    off();
}

void SteppingMotor::drive(int phase) {
  extern int verbose;
  if (verbose) log_i("%d", phase);
  _bits = drv_pat[phase];
  _bOffed = false;
}

void SteppingMotor::off(void) {
  _bits = 0;
  _bOffed = true;
}

void SteppingMotor::start(void) {
  drive(_phase);
  if (_tPeriod > 0) {
    _tUpdate = micros() + _tPeriod;
    _bRunning = true;
  }
}

void SteppingMotor::stop(void) {
  _bRunning = false;
  _tStop = micros();
}

void SteppingMotor::setRPM(float rpm) {
  if (rpm < 0) {
    _bRev = true;
    rpm = -rpm;
  } else if (rpm > 0) {
    _bRev = false;
  } else {
    _bRunning = false;
    _tStop = micros();
  }
  _rpm = rpm;
  _tPeriod = 60 * 1000000 / _stepPerRot / rpm;
  _bForever = true;
  log_i("period: %luus", _tPeriod);
}

void SteppingMotor::setPos(int32_t pos) {
  _bRev = pos < 0;
  _bForever = false;
  _targetPos = pos;
}

I2CSteppingMotorDriver::I2CSteppingMotorDriver(TwoWire *wire, uint8_t addr) {
  _wire = wire;
  _addr = addr;
  _bError = false;
}

i2c_err_t I2CSteppingMotorDriver::readReg(uint8_t addr, uint8_t *pData) {
  _wire->beginTransmission(_addr);
  _wire->write(addr);
  i2c_err_t r = (i2c_err_t)_wire->endTransmission(false);
  if (r != I2C_ERROR_OK) return r;
  uint8_t n = _wire->requestFrom((int)_addr, 1);
  if (n != 1) return (i2c_err_t)_wire->lastError();
  *pData = _wire->read();
  return I2C_ERROR_OK;
}

i2c_err_t I2CSteppingMotorDriver::writeReg(uint8_t addr, uint8_t data) {
  _wire->beginTransmission(_addr);
  _wire->write(addr);
  _wire->write(data);
  return (i2c_err_t)_wire->endTransmission();
}

#define IODIRA 0x00
#define IODIRB 0x01
#define IPOLA 0x02
#define IOPOLB 0x03
#define GPINTENA 0x04
#define GPINTENB 0x05
#define DEFVALA 0x06
#define DEFVALB 0x07
#define INTCONA 0x08
#define INTCONB 0x09
#define IOCONA 0x0A
#define IOCONB 0x0B
#define GPPUA 0x0C
#define GPPUB 0x0D
#define INTFA 0x0E
#define INTFB 0x0F
#define INTCAPA 0x10
#define INTCAPB 0x11
#define GPIOA 0x12
#define GPIOB 0x13
#define OLATA 0x14
#define OLATB 0x15

void I2CSteppingMotorDriver::init(void) {
  writeReg(IODIRA, 0x00);  // PORT-A is output
  writeReg(IODIRB, 0xff);  // PORT-B is input
  writeReg(GPPUB, 0xff);   // PORT-B pullup enabled
  writeReg(OLATA, 0x00);
  _bits = 0;
  _sm[0].init();
  _sm[1].init();
}

void I2CSteppingMotorDriver::update(void) {
  uint8_t bits = 0;
  _sm[0].update();
  _sm[1].update();
  bits = (_sm[0]._bits << 4) | _sm[1]._bits;
  if (bits != _bits) {
    extern int verbose;
    if (verbose) log_i("bits: 0x%02x", bits);
    writeReg(OLATA, bits);
    _bits = bits;
  }
}