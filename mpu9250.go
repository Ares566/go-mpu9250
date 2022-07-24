package go_mpu9250

import (
	"errors"
	"github.com/Ares566/go-mpu9250/pkg/reg"
	_i2c "github.com/d2r2/go-i2c"
	"github.com/d2r2/go-logger"
	"time"
)

const (
	registers        = 12
	accelsenSitivity = 16384
)

var (
	initSequence = [][]byte{
		{reg.MPU9250_PWR_MGMT_1, 0x0}, // Clear sleep mode bit (6), enable all sensors
		{100},
		{reg.MPU9250_PWR_MGMT_2, 0x01}, // Auto select clock source to be PLL gyroscope reference if ready else
		{100},
		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
		// be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
		{reg.MPU9250_CONFIG, 0x03},
		{reg.MPU9250_SMPLRT_DIV, 0x04}, // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	}
	calibrateSequence = [][]byte{
		{reg.MPU9250_PWR_MGMT_1, 0x80}, // reset device
		{100},                          // sleep 100 ms
		{reg.MPU9250_PWR_MGMT_1, 1},    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready else use the internal oscillator, bits 2:0 = 001
		{reg.MPU9250_PWR_MGMT_2, 0},
		{200},                         // wait 200 ms
		{reg.MPU9250_INT_ENABLE, 0},   // Disable all interrupts
		{reg.MPU9250_FIFO_EN, 0},      // Disable FIFO
		{reg.MPU9250_PWR_MGMT_1, 0},   // Turn on internal clock source
		{reg.MPU9250_I2C_MST_CTRL, 0}, // Disable I2C master
		{reg.MPU9250_USER_CTRL, 0},    // Disable FIFO and I2C master modes
		{reg.MPU9250_USER_CTRL, 0x0C}, // Reset FIFO and DMP
		{15},                          // wait 15 ms
		{reg.MPU9250_CONFIG, 0x01},    // Set low-pass filter to 188 Hz
		{reg.MPU9250_SMPLRT_DIV, 0},   // Set sample rate to 1 kHz
		{reg.MPU9250_GYRO_CONFIG, 0},  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
		{reg.MPU9250_ACCEL_CONFIG, 0}, // Set accelerometer full-scale to 2 g, maximum sensitivity
		{reg.MPU9250_USER_CTRL, 0x40}, // Enable FIFO
		{reg.MPU9250_FIFO_EN, 0x78},   // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
		{40},                          // wait 40 ms
		{reg.MPU9250_FIFO_EN, 0x00},   // Disable gyro and accelerometer sensors for FIFO
	}
)

// AccelerometerData the values for x/y/z axises.
type AccelerometerData struct {
	X, Y, Z int16
}

// GyroscopeData the values for x/y/z axises.
type GyroscopeData struct {
	X, Y, Z int16
}

// RotationData the rotation around X/Y/Z axises.
type RotationData struct {
	X, Y, Z int16
}

// Deviation defines the standard deviation for major axises.
type Deviation struct {
	X, Y, Z float64
}

// SelfTestResult defines the results for self-test for accelerometer, gyroscope.
type SelfTestResult struct {
	AccelDeviation Deviation
	GyroDeviation  Deviation
}

// MPU9250 defines the structure to keep reference to the transport.
type MPU9250 struct {
	i2c *_i2c.I2C
	log logger.PackageLog
}

// Calibrate calibrates the device using maximum precision for both Gyroscope
// and Accelerometer.
func (m *MPU9250) Calibrate() error {
	if err := m.transferBatch(calibrateSequence, "error calibrating %d: [%x:%x] => %v"); err != nil {
		return err
	}
	reads, err := m.GetFIFOCount() // read FIFO sample count
	if err != nil {
		m.log.Errorf("can't get FIFO => %v", err)
		return err
	}

	m.log.Debugf("Read %d packets\n", reads)

	packets := reads / registers

	var buffer [registers]byte

	toUint16 := func(offset int) int16 {
		return int16(buffer[offset])<<8 | int16(buffer[offset+1])
	}

	writeGyroOffset := func(v int16, h, l byte) error {
		o := (-v) >> 2
		if err = m.i2c.WriteRegU8(h, byte(o>>8)); err != nil {
			m.log.Errorf("can't write Gyro offset %x(h):%x => %w", h, o, err)
			return err
		}
		if err = m.i2c.WriteRegU8(l, byte(o&0xFF)); err != nil {
			m.log.Errorf("can't write Gyro offset %x(l):%x => %w", l, o, err)
			return err

		}
		return nil
	}

	writeAccelOffset := func(o uint16, h, l byte) error {
		if err = m.i2c.WriteRegU8(h, byte(o>>8)); err != nil {
			m.log.Errorf("can't write Accelerator %x(l):%x => %w", l, o, err)
			return err
		}
		if err = m.i2c.WriteRegU8(l, byte(o&0xFF)); err != nil {
			m.log.Errorf("can't write Accelerator %x(l):%x => %w", l, o, err)
			return err
		}
		return nil
	}

	var (
		accelX, accelY, accelZ, gyroX, gyroY, gyroZ                         int64
		accelXBias, accelYBias, accelZBias, gyroXBias, gyroYBias, gyroZBias int16
	)

	for i := 0; i < int(packets); i++ {
		for j := 0; j < registers; j++ {
			b := byte(0)
			if b, err = m.GetFIFOByte(); err != nil {
				m.log.Errorf("can't read data byte %d of packet %d => %w", j, i, err)
				return err
			}
			buffer[j] = b
		}
		accelX += int64(toUint16(0))
		accelY += int64(toUint16(2))
		accelZ += int64(toUint16(4))
		gyroX += int64(toUint16(6))
		gyroY += int64(toUint16(8))
		gyroZ += int64(toUint16(10))
	}

	accelXBias = int16(accelX / int64(packets))
	accelYBias = int16(accelY / int64(packets))
	accelZBias = int16(accelZ / int64(packets))
	gyroXBias = int16(gyroX / int64(packets))
	gyroYBias = int16(gyroY / int64(packets))
	gyroZBias = int16(gyroZ / int64(packets))

	m.log.Debugf("Raw accelerometer bias: X:%d, Y:%d, Z:%d\n", accelXBias, accelYBias, accelZBias)
	m.log.Debugf("Raw gyroscope bias X:%d, Y:%d, Z:%d\n", gyroXBias, gyroYBias, gyroZBias)

	if accelZBias > 0 {
		accelZBias -= accelsenSitivity
	} else {
		accelZBias += accelsenSitivity
	}

	var factoryGyroBiasX, factoryGyroBiasY, factoryGyroBiasZ int16
	factoryGyroBiasX, err = m.ReadSignedWord(reg.MPU9250_XG_OFFSET_H, reg.MPU9250_XG_OFFSET_L)
	if err != nil {
		return err
	}
	factoryGyroBiasY, err = m.ReadSignedWord(reg.MPU9250_YG_OFFSET_H, reg.MPU9250_YG_OFFSET_L)
	if err != nil {
		return err
	}
	factoryGyroBiasZ, err = m.ReadSignedWord(reg.MPU9250_ZG_OFFSET_H, reg.MPU9250_ZG_OFFSET_L)
	if err != nil {
		return err
	}
	m.log.Debugf("Factory gyroscope bias: X:%d, Y:%d, Z:%d\n", int16(factoryGyroBiasX), int16(factoryGyroBiasY), int16(factoryGyroBiasZ))

	if err = writeGyroOffset(gyroXBias, reg.MPU9250_GYRO_XOUT_H, reg.MPU9250_GYRO_XOUT_L); err != nil {
		return err
	}
	if err = writeGyroOffset(gyroYBias, reg.MPU9250_GYRO_YOUT_H, reg.MPU9250_GYRO_YOUT_L); err != nil {
		return err
	}
	if err = writeGyroOffset(gyroZBias, reg.MPU9250_GYRO_ZOUT_H, reg.MPU9250_GYRO_ZOUT_L); err != nil {
		return err
	}

	// Construct the accelerometer biases for push to the hardware accelerometer
	// bias registers. These registers contain factory trim values which must be
	// added to the calculated accelerometer biases; on boot up these registers
	// will hold non-zero values.
	var factoryBiasX, factoryBiasY, factoryBiasZ int16
	factoryBiasX, err = m.ReadSignedWord(reg.MPU9250_XA_OFFSET_H, reg.MPU9250_XA_OFFSET_L)
	if err != nil {
		return err
	}
	factoryBiasY, err = m.ReadSignedWord(reg.MPU9250_YA_OFFSET_H, reg.MPU9250_YA_OFFSET_L)
	if err != nil {
		return err
	}
	factoryBiasZ, err = m.ReadSignedWord(reg.MPU9250_ZA_OFFSET_H, reg.MPU9250_ZA_OFFSET_L)
	if err != nil {
		return err
	}

	// In addition, bit 0 of the lower byte must be preserved since it is used
	// for temperature compensation calculations.
	maskX := factoryBiasX & 1
	maskY := factoryBiasY & 1
	maskZ := factoryBiasZ & 1

	m.log.Debugf("Factory accelerometer bias: X:%d, Y:%d, Z:%d\n", int16(factoryBiasX), int16(factoryBiasY), int16(factoryBiasZ))

	// Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.
	factoryBiasX -= accelXBias >> 3
	factoryBiasY -= accelYBias >> 3
	factoryBiasZ -= accelZBias >> 3

	// restore the temperature preserve bit
	factoryBiasX |= maskX
	factoryBiasY |= maskY
	factoryBiasZ |= maskZ

	if err := writeAccelOffset(uint16(factoryBiasX), reg.MPU9250_XA_OFFSET_H, reg.MPU9250_XA_OFFSET_L); err != nil {
		return err
	}
	if err := writeAccelOffset(uint16(factoryBiasY), reg.MPU9250_YA_OFFSET_H, reg.MPU9250_YA_OFFSET_L); err != nil {
		return err
	}
	return writeAccelOffset(uint16(factoryBiasZ), reg.MPU9250_ZA_OFFSET_H, reg.MPU9250_ZA_OFFSET_L)
}
func (m *MPU9250) GetFIFOByte() (byte, error) {
	return m.i2c.ReadRegU8(reg.MPU9250_FIFO_R_W)
}
func (m *MPU9250) GetFIFOCount() (uint16, error) {
	return m.readWord(reg.MPU9250_FIFO_COUNTH, reg.MPU9250_FIFO_COUNTL)
}

// SetGyroRange sets the gyroscope range.
//
// The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
// as described in the table below.
//
//  0 = +/- 250 degrees/sec
//  1 = +/- 500 degrees/sec
//  2 = +/- 1000 degrees/sec
//  3 = +/- 2000 degrees/sec
func (m *MPU9250) SetGyroRange(rangeVal byte) error {
	if rangeVal > 3 {
		return errors.New("accepted values are in the range 0 .. 3")
	}
	return m.writeMaskedReg(reg.MPU9250_GYRO_CONFIG, reg.MPU9250_GYRO_FS_SEL_MASK, rangeVal<<3)
}

// GetGyroRange gets the gyroscope range.
func (m *MPU9250) GetGyroRange() (byte, error) {
	b, err := m.readMaskedReg(reg.MPU9250_GYRO_CONFIG, reg.MPU9250_GYRO_FS_SEL_MASK)
	return b >> 3, err // the mask is 11000
}

// SetAccelRange sets the full-scale accelerometer range.
//
// The FS_SEL parameter allows setting the full-scale range of the accelerometer
// sensors, as described in the table below.
//
//  0 = +/- 2g
//  1 = +/- 4g
//  2 = +/- 8g
//  3 = +/- 16g
func (m *MPU9250) SetAccelRange(rangeVal byte) error {
	if (rangeVal >> 3) > 3 {
		return errors.New("accepted values are in the range 0 .. 3")
	}
	return m.writeMaskedReg(reg.MPU9250_ACCEL_CONFIG, reg.MPU9250_ACCEL_FS_SEL_MASK, rangeVal)
}

// GetAccelRange gets the full-scale accelerometer range.
func (m *MPU9250) GetAccelRange() (byte, error) {
	b, err := m.readMaskedReg(reg.MPU9250_ACCEL_CONFIG, reg.MPU9250_ACCEL_FS_SEL_MASK)
	return b >> 3, err
}

// AccelerometerXIsEnabled gets the X accelerometer status.
func (m *MPU9250) AccelerometerXIsEnabled() (bool, error) {
	return negateBool(m.readMaskedRegBool(reg.MPU9250_PWR_MGMT_2, reg.MPU9250_DISABLE_XA_MASK))
}

// AccelerometerYIsEnabled gets the X accelerometer status.
func (m *MPU9250) AccelerometerYIsEnabled() (bool, error) {
	return negateBool(m.readMaskedRegBool(reg.MPU9250_PWR_MGMT_2, reg.MPU9250_DISABLE_YA_MASK))
}

// AccelerometerZIsEnabled gets the Z accelerometer status.
func (m *MPU9250) AccelerometerZIsEnabled() (bool, error) {
	return negateBool(m.readMaskedRegBool(reg.MPU9250_PWR_MGMT_2, reg.MPU9250_DISABLE_ZA_MASK))
}

// GetAccelerationX reads the X-axis accelerometer.
func (m *MPU9250) GetAccelerationX() (int16, error) {
	if enabled, err := m.AccelerometerXIsEnabled(); err == nil && enabled {
		return uintToInt(m.readWord(reg.MPU9250_ACCEL_XOUT_H, reg.MPU9250_ACCEL_XOUT_L))
	} else if !enabled {
		m.log.Error("X acceleration disabled", err)
		return 0, err
	} else {
		return 0, err
	}
}

// GetAccelerationY reads the Y-axis accelerometer.
func (m *MPU9250) GetAccelerationY() (int16, error) {
	if enabled, err := m.AccelerometerYIsEnabled(); err == nil && enabled {
		return uintToInt(m.readWord(reg.MPU9250_ACCEL_YOUT_H, reg.MPU9250_ACCEL_YOUT_L))
	} else if !enabled {
		m.log.Error("Y acceleration disabled", err)
		return 0, err
	} else {
		return 0, err
	}
}

// GetAccelerationZ reads the Z-axis accelerometer.
func (m *MPU9250) GetAccelerationZ() (int16, error) {
	if enabled, err := m.AccelerometerZIsEnabled(); err == nil && enabled {
		return uintToInt(m.readWord(reg.MPU9250_ACCEL_ZOUT_H, reg.MPU9250_ACCEL_ZOUT_L))
	} else if !enabled {
		m.log.Error("Z acceleration disabled", err)
		return 0, err
	} else {
		return 0, err
	}
}

// Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
// (Register 28). For each full scale setting, the accelerometers' sensitivity
// per LSB in ACCEL_xOUT is shown in the table below:
//
//  AFS_SEL | Full Scale Range | LSB Sensitivity
//  --------+------------------+----------------
//  0       | +/- 2g           | 8192 LSB/mg
//  1       | +/- 4g           | 4096 LSB/mg
//  2       | +/- 8g           | 2048 LSB/mg
//  3       | +/- 16g          | 1024 LSB/mg
func (m *MPU9250) GetAcceleration() (*AccelerometerData, error) {
	x, err := m.GetAccelerationX()
	if err != nil {
		return nil, err
	}
	y, err := m.GetAccelerationY()
	if err != nil {
		return nil, err
	}
	z, err := m.GetAccelerationZ()
	if err != nil {
		return nil, err
	}
	return &AccelerometerData{X: x, Y: y, Z: z}, nil
}

// GyroXIsEnabled gets the X axis enabled flag.
func (m *MPU9250) GyroXIsEnabled() (bool, error) {
	return negateBool(m.readMaskedRegBool(reg.MPU9250_PWR_MGMT_2, reg.MPU9250_DISABLE_XG_MASK))
}

// GyroYIsEnabled gets the Y axis enabled flag.
func (m *MPU9250) GyroYIsEnabled() (bool, error) {
	return negateBool(m.readMaskedRegBool(reg.MPU9250_PWR_MGMT_2, reg.MPU9250_DISABLE_YG_MASK))
}

// GyroZIsEnabled gets the Z axis enabled flag.
func (m *MPU9250) GyroZIsEnabled() (bool, error) {
	return negateBool(m.readMaskedRegBool(reg.MPU9250_PWR_MGMT_2, reg.MPU9250_DISABLE_ZG_MASK))
}

// GetRotationX gets X-axis gyroscope reading.
func (m *MPU9250) GetRotationX() (int16, error) {
	if enabled, err := m.GyroXIsEnabled(); enabled && err == nil {
		return asInt16(m.readWord(reg.MPU9250_GYRO_XOUT_H, reg.MPU9250_GYRO_XOUT_L))
	} else if !enabled {
		m.log.Error("X rotation disabled", err)
		return 0, err
	} else {
		return 0, err
	}
}

// GetRotationY gets Y-axis gyroscope reading.
func (m *MPU9250) GetRotationY() (int16, error) {
	if enabled, err := m.GyroYIsEnabled(); enabled && err == nil {
		return asInt16(m.readWord(reg.MPU9250_GYRO_YOUT_H, reg.MPU9250_GYRO_YOUT_L))
	} else if !enabled {
		m.log.Error("Y rotation disabled", err)
		return 0, err
	} else {
		return 0, err
	}
}

// GetRotationZ gets Z-axis gyroscope reading.
func (m *MPU9250) GetRotationZ() (int16, error) {
	if enabled, err := m.GyroZIsEnabled(); enabled && err == nil {
		return asInt16(m.readWord(reg.MPU9250_GYRO_ZOUT_H, reg.MPU9250_GYRO_ZOUT_L))
	} else if !enabled {
		m.log.Error("Z rotation disabled", err)
		return 0, err
	} else {
		return 0, err
	}
}

// Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
// (Register 27). For each full scale setting, the gyroscopes' sensitivity per
// LSB in GYRO_xOUT is shown in the table below:
//
//  FS_SEL | Full Scale Range   | LSB Sensitivity
//  -------+--------------------+----------------
//  0      | +/- 250 degrees/s  | 131 LSB/deg/s
//  1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
//  2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
//  3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
func (m *MPU9250) GetRotation() (*RotationData, error) {
	x, err := m.GetRotationX()
	if err != nil {
		return nil, err
	}
	y, err := m.GetRotationY()
	if err != nil {
		return nil, err
	}
	z, err := m.GetRotationZ()
	if err != nil {
		return nil, err
	}
	return &RotationData{X: x, Y: y, Z: z}, nil
}

// GetMotion6 gets the motion data - accelerometer and rotation(gyroscope).
func (m *MPU9250) GetMotion6() (*AccelerometerData, *RotationData, error) {
	acc, err := m.GetAcceleration()
	if err != nil {
		return nil, nil, err
	}
	rot, err := m.GetRotation()
	if err != nil {
		return nil, nil, err
	}
	return acc, rot, nil
}

// TemperatureIsEnabled returns if the temperature sensor enabled.
func (m *MPU9250) TemperatureIsEnabled() (bool, error) {
	return m.readMaskedRegBool(reg.MPU9250_FIFO_EN, reg.MPU9250_TEMP_FIFO_EN_MASK)
}

// GetTemperature gets the current temperature.
func (m *MPU9250) GetTemperature() (float64, error) {
	// TODO: Use physic.Kelvin.
	//((Temp_out - room_temp_offset)/temp_sensitivity) + 21; //celcius
	if enabled, err := m.TemperatureIsEnabled(); enabled && err != nil {
		return 0, err
	}
	T, err := m.readWord(reg.MPU9250_TEMP_OUT_H, reg.MPU9250_TEMP_OUT_L)
	if err != nil {
		return 0, err
	}
	tC := float64(T)/333.87 + 21.0
	return tC, nil
}

func (m *MPU9250) readMaskedRegBool(address, mask byte) (bool, error) {
	response, err := m.readMaskedReg(address, mask)
	if err != nil {
		return false, err
	}
	return response != 0, nil
}
func (m *MPU9250) readMaskedReg(address byte, mask byte) (byte, error) {

	v, err := m.i2c.ReadRegU8(address)
	if err != nil {
		return 0, err
	}
	return v & mask, nil
}

func (m *MPU9250) readWord(hi, lo byte) (uint16, error) {

	hiByte, err := m.i2c.ReadRegU8(hi)
	if err != nil {
		return 0, err
	}
	loByte, err := m.i2c.ReadRegU8(lo)
	if err != nil {
		return 0, err
	}
	return uint16(hiByte)<<8 | uint16(loByte), nil
}

func (m *MPU9250) ReadSignedWord(hi, lo byte) (int16, error) {
	res, err := m.readWord(hi, lo)
	return int16(res), err
}

func (m *MPU9250) writeByteAddress(address, value byte) error {
	return m.i2c.WriteRegU8(address, value)
}

func (m *MPU9250) writeMaskedReg(address byte, mask byte, value byte) error {
	maskedValue := mask & value
	regVal, err := m.i2c.ReadRegU8(address)
	if err != nil {
		return err
	}
	regVal = (regVal &^ maskedValue) | maskedValue
	return m.i2c.WriteRegU8(address, regVal)
}

func negateBool(src bool, err error) (bool, error) {
	if err != nil {
		return src, err
	}
	return !src, err
}
func asInt16(src uint16, err error) (int16, error) {
	return int16(src), err
}
func uintToInt(s uint16, err error) (int16, error) {
	return int16(s), err
}
func (m *MPU9250) transferBatch(seq [][]byte, msg string) error {
	for i, cmds := range seq {
		if len(cmds) == 2 {
			if err := m.i2c.WriteRegU8(cmds[0], cmds[1]); err != nil {
				m.log.Errorf(msg, i, cmds[0], cmds[1], err)
				return err
			}
		} else {
			time.Sleep(time.Duration(cmds[0]) * time.Millisecond)
		}
	}
	return nil
}

// Init initializes the device.
func (m *MPU9250) Init() error {
	return m.transferBatch(initSequence, "error initializing %d: [%x:%x] => %v")
}

// New creates the new instance of the driver.
//
// transport the transport interface.
func New(i2c *_i2c.I2C, log logger.PackageLog) *MPU9250 {
	return &MPU9250{i2c, log}
}
