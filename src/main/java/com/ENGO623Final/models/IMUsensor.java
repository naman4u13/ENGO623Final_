package com.ENGO623Final.models;

public class ImuSensor {

	private double time;
	private double accX;
	private double accY;
	private double accZ;
	private double gyroX;
	private double gyroY;
	private double gyroZ;

	public ImuSensor(double[] data) {
		super();
		this.time = data[0];
		this.gyroX = data[1];
		this.gyroY = data[2];
		this.gyroZ = data[3];
		this.accX = data[4];
		this.accY = data[5];
		this.accZ = data[6];
	}

	public double getTime() {
		return time;
	}

	public double getAccX() {
		return accX;
	}

	public double getAccY() {
		return accY;
	}

	public double getAccZ() {
		return accZ;
	}

	public double getGyroX() {
		return gyroX;
	}

	public double getGyroY() {
		return gyroY;
	}

	public double getGyroZ() {
		return gyroZ;
	}

	public double[] getAcc() {
		return new double[] { accX, accY, accZ };
	}

	public double[] getGyro() {
		return new double[] { gyroX, gyroY, gyroZ };
	}

}
