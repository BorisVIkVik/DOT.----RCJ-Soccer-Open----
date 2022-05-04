//bool initIMU_(void)
//{
//  // imu.begin() should return 0 on success. Will initialize
//  // I2C bus, and reset MPU-9250 to defaults.
//  if (imu.init(IMU_SPI, IMU_SPI_SS, IMU_PWR_EN);)
//    return false;

//  // Set up MPU-9250 interrupt:
//  imu.enableInterrupt(); // Enable interrupt output
//  imu.setIntLevel(1);    // Set interrupt to active-low
//  imu.setIntLatched(1);  // Latch interrupt output

//  // Configure sensors:
//  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
//  imu.setGyroFSR(gyroFSR);
//  // Set accel full-scale range: options are 2, 4, 8, or 16 g 
//  imu.setAccelFSR(accelFSR);
//  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
//  imu.setLPF(IMU_AG_LPF); 
//  // Set gyro/accel sample rate: must be between 4-1000Hz
//  // (note: this value will be overridden by the DMP sample rate)
//  imu.setSampleRate(IMU_AG_SAMPLE_RATE); 
//  // Set compass sample rate: between 4-100Hz
//  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE); 

//  // Configure digital motion processor. Use the FIFO to get
//  // data from the DMP.
//  unsigned short dmpFeatureMask = 0;
//  if (ENABLE_GYRO_CALIBRATION)
//  {
//    // Gyro calibration re-calibrates the gyro after a set amount
//    // of no motion detected
//    dmpFeatureMask |= DMP_FEATURE_GYRO_CAL;
//    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
//  }
//  else
//  {
//    // Otherwise add raw gyro readings to the DMP
//    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
//  }
//  // Add accel and quaternion's to the DMP
//  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
//  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

//  // Initialize the DMP, and set the FIFO's update rate:
//  imu.dmpBegin(dmpFeatureMask, fifoRate);

//  return true; // Return success
//}

//inv_error_t MPU9250_DMP::dmpBegin(unsigned short features, unsigned short fifoRate)
//{
//	unsigned short feat = features;
//	unsigned short rate = fifoRate;

//	if (dmpLoad() != INV_SUCCESS)
//		return INV_ERROR;
//	
//	// 3-axis and 6-axis LP quat are mutually exclusive.
//	// If both are selected, default to 3-axis
//	if (feat & DMP_FEATURE_LP_QUAT)
//	{
//		feat &= ~(DMP_FEATURE_6X_LP_QUAT);
//		dmp_enable_lp_quat(1);
//	}
//	else if (feat & DMP_FEATURE_6X_LP_QUAT)
//		dmp_enable_6x_lp_quat(1);
//	
//	if (feat & DMP_FEATURE_GYRO_CAL)
//		dmp_enable_gyro_cal(1);
//	
//	if (dmpEnableFeatures(feat) != INV_SUCCESS)
//		return INV_ERROR;
//	
//	rate = constrain(rate, 1, 200);
//	if (dmpSetFifoRate(rate) != INV_SUCCESS)
//		return INV_ERROR;
//	
//	return mpu_set_dmp_state(1);
//}

//static struct gyro_state_s st = {
//    .reg = &reg,
//    .hw = &hw,
//    .test = &test
//};

//int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
//    unsigned short start_addr, unsigned short sample_rate)
//{
//    unsigned short ii;
//    unsigned short this_write;
//    /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
//#define LOAD_CHUNK  (16)
//    unsigned char cur[LOAD_CHUNK], tmp[2];

//    if (st.chip_cfg.dmp_loaded)
//        /* DMP should only be loaded once. */
//        return -1;

//    if (!firmware)
//        return -1;
//    for (ii = 0; ii < length; ii += this_write) {
//        this_write = min(LOAD_CHUNK, length - ii);
//        if (mpu_write_mem(ii, this_write, (unsigned char*)&firmware[ii]))
//            return -1;
//        if (mpu_read_mem(ii, this_write, cur))
//            return -1;
//        if (memcmp(firmware+ii, cur, this_write))
//            return -2;
//    }

//    /* Set program start address. */
//    tmp[0] = start_addr >> 8;
//    tmp[1] = start_addr & 0xFF;
//    if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp))
//        return -1;

//    st.chip_cfg.dmp_loaded = 1;
//    st.chip_cfg.dmp_sample_rate = sample_rate;
//    return 0;
//}