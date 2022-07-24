package main

import (
	mpu9250 "github.com/Ares566/go-mpu9250"
	i2c "github.com/d2r2/go-i2c"
	"github.com/d2r2/go-logger"
)

var lg = logger.NewPackageLogger("main",
	logger.DebugLevel,
	// logger.InfoLevel,
)

func main() {
	defer logger.FinalizeLogger()
	// Create new connection to i2c-bus on 1 line with address
	//mpu, err := i2c.NewI2C(0x76, 1) //BMP280
	mpu, err := i2c.NewI2C(0x68, 1) //MPU9250
	if err != nil {
		lg.Fatal(err)
	}
	defer mpu.Close()

	lg.Info(mpu)

	mpuDriver := mpu9250.New(mpu, lg)
	mpuDriver.Init()
	err = mpuDriver.Calibrate()
	if err != nil {
		lg.Fatal(err)
	}

}
