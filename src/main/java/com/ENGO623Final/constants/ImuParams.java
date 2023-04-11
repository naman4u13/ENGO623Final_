package com.ENGO623Final.constants;

public class ImuParams {

	private final static double degPerHr_2_radPerS = (Math.PI / (180 * 60 * 60));
	private final static double degPerSqrHr_2_radPerSqrS = (Math.PI / (180 * 60));
	// in rad/s
	public static final double gyro_bias = 0.1 * degPerHr_2_radPerS;
	// in rad/(s^0.5)
	public static final double ARW = 0.01 * degPerSqrHr_2_radPerSqrS;
	// in rad/s
	public static final double gyro_bias_instability = 0.015 * degPerHr_2_radPerS;
	// in sec
	public static final double gyro_corr_time = 1 * 60 * 60;
	// in m/(s^2)
	public static final double acc_bias = 3*1e-6;
	// in m/(s^1.5)
	public static final double VRW = 0.003/60;
	// in m/(s^2)
	public static final double acc_bias_instability = 50*1e-6;
	// in sec
	public static final double acc_corr_time = 1 * 60 * 60;
}
