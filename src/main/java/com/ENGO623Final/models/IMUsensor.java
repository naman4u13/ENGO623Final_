package com.ENGO623Final.models;

public class IMUsensor {

	private double time;
	private double accX;
	private double accY;
	private double accZ;
	private double gyroX;
	private double gyroY;
	private double gyroZ;
	public IMUsensor(double[] data) {
		super();
		this.time = data[0];
		this.accX = data[1];
		this.accY = data[2];
		this.accZ = data[3];
		this.gyroX =data[4];
		this.gyroY =data[5];
		this.gyroZ =data[6];
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
	
	
	
}
