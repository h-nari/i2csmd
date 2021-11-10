#include <Wire.h>
#include <Adafruit_MCP23X17.h>

class SteppingMotor {
 public:
  uint8_t _bits;

 private:
  uint8_t _phase;
  bool _bRunning;
  bool _bRev;
  bool _bForever;
  bool _bOffed;
  uint32_t _stepPerRot;
  float _rpm;  // [rpm]
  int32_t _curPos;
  int32_t _targetPos;
  uint32_t _tPeriod;
  uint32_t _tUpdate;
  uint32_t _tStop;

 public:
  SteppingMotor();
  void init(void);
  void update(void);
  void setRPM(float rpm);
  void setPos(int32_t pos);
  void stop(void);
  void start(void);
  void off(void);
  bool isRunning(void) { return _bRunning; }
  bool getPos(void) { return _curPos; }
  float getRpm(void) { return _bRev ? -_rpm : _rpm; }

 private:
  void drive(int phase);
};

class I2CSteppingMotorDriver {
 public:
  SteppingMotor _sm[2];

 private:
  Adafruit_MCP23X17 _mcp;
  bool _bError;
  uint8_t _addr;
  uint8_t _bits;
  TwoWire *_wire;

 public:
  I2CSteppingMotorDriver(TwoWire *wire = &Wire, uint8_t addr = 0x20);
  i2c_err_t readReg(uint8_t addr, uint8_t *pData);
  i2c_err_t writeReg(uint8_t addr, uint8_t data);
  void init(void);
  void update(void);
};
