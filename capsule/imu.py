import adafruit_bno055

class SensorSet:

    IMU_BUS = 0x28

    def __init__(self, i2c, logger) -> None:
        """Create a sensor set composed by a BNO055 IMU

        Args:
            i2c (I2C): i2c bus object from the Beaglebone Black
            logger (Logger): data logger
        """
        # assign fields for the class instance
        self.i2c = i2c
        self.logger = logger
        
        # proceed to initialize Altimeter, IMU, and determine altitude on pad
        sensorset_init_message = "SS-INIT\n"
        sensorset_init_message += self.__imu_reset(i2c)
        logger.log(sensorset_init_message)

    def read_data(self):
        """acquire packaged z-axis accelerations and altitude

        Returns:
            float: altitude measurement
            float: z-axis acceleration
            float: z-axis linear acceleration
            bool: altimeter data availability
            bool: IMU data availability
        """
        logger = self.logger
        # diagnose sensor health and reset sensors if necessary
        if not self.healthy_imu:
            self.__imu_reset(self.i2c)

        # attempt to read data from sensors
        za, lza = self.read_acceleration()
        logger.log(f"SS-READ:ZACC-{za}|LZAC-{lza}|HEATI-{self.healthy_alti}|HEIMU-{self.healthy_imu}")
        return za, lza, self.healthy_imu

    def read_acceleration(self):
        """read measurement from IMU accelerometer

        Returns:
            float: z-axis acceleration
            float: z-axis linear acceleration
        """
        imu = self.imu
        logger = self.logger
        # obtain z-axis acceleration from IMU
        try:
            _, _, za = imu.acceleration
            _, _, lza = imu.linear_acceleration
            self.za = za
            self.lza = lza
        # log exceptions reported
        except Exception as err:
            za, lza = None, None
            logger.log(f"IMU-RDER:{err}")
            self.healthy_imu = False

        return za, lza
    
    def __imu_reset(self, i2c):
        """initialize the IMU by re-assigning an IMU instance

        Args:
            i2c (I2C): i2c bus object from the Beaglebone Black

        Returns:
            str: IMU initialization message
        """
        # reset IMU by reconstruct an IMU object
        try:
            imu = adafruit_bno055.BNO055_I2C(i2c, address=SensorSet.IMU_BUS)
            imu_reset_message = "IMU-SUCCESS\n"
            healthy_imu = True
        # log exceptions reported
        except Exception as err:
            imu = None
            imu_reset_message = f"IMU-FAIL-ERR:{err}\n"
            healthy_imu = False
        self.imu = imu
        self.healthy_imu = healthy_imu

        return imu_reset_message
