package com.ENGO623Final.helper;

import java.util.ArrayList;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

import com.ENGO623Final.Util.LatLonUtil;
import com.ENGO623Final.Util.Rotation;
import com.ENGO623Final.models.ImuSensor;

public class InitialAlignment {

	// Self-Alignment
	// Refer Paul D. Groves Book
	public static SimpleMatrix process(ArrayList<ImuSensor> dataList, int m)
	{
		double[] avgAcc = new double[3];
		double[] avgGyro = new double[3];
		for(int i=0;i<m;i++)
		{
			avgAcc[0] += dataList.get(i).getAccX();
			avgAcc[1] += dataList.get(i).getAccY();
			avgAcc[2] += dataList.get(i).getAccZ();
			avgGyro[0] += dataList.get(i).getGyroX();
			avgGyro[1] += dataList.get(i).getGyroY();
			avgGyro[2] += dataList.get(i).getGyroZ();
		}
		IntStream.range(0,3).forEach(i->avgAcc[i]/=m);
		IntStream.range(0,3).forEach(i->avgGyro[i]/=m);
		// Accelerometer Leveling
		double pitch = Math.atan(-avgAcc[0]/Math.sqrt(Math.pow(avgAcc[1], 2)+Math.pow(avgAcc[2], 2)));
		double roll = Math.atan2(-avgAcc[1], -avgAcc[2]);
		
		// Gyro-Compassing
		double sin_yaw = (-avgGyro[1]*Math.cos(roll)) + (avgGyro[2]*Math.sin(roll));
		double cos_yaw = (avgGyro[0]*Math.cos(pitch)) + (avgGyro[1]*Math.sin(roll)*Math.sin(pitch))+(avgGyro[2]*Math.cos(roll)*Math.sin(pitch));
		double yaw = Math.atan2(sin_yaw, cos_yaw);
		
		double g = LatLonUtil.getGravity(Math.toRadians(51.07995352), 1118.502);
		double r = Math.signum(avgAcc[2])*Math.asin(avgAcc[1]/g);
		double p = -Math.signum(avgAcc[2])*Math.asin(avgAcc[0]/g);
		double y = Math.atan2(avgGyro[1],avgGyro[0]);
		SimpleMatrix dcm = new SimpleMatrix(Rotation.euler2dcm(new double[] {roll,pitch,yaw}));
		SimpleMatrix _dcm = new SimpleMatrix(Rotation.euler2dcm(new double[] {r,p,y}));
		dcm = Rotation.reorthonormDcm(_dcm);
		return dcm;
	}
	
	
}
