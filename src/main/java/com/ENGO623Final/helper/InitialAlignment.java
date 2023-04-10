package com.ENGO623Final.helper;

import java.util.ArrayList;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

import com.ENGO623Final.Util.Rotation;
import com.ENGO623Final.models.IMUsensor;

public class InitialAlignment {

	// Self-Alignment
	// Refer Paul D. Groves Book
	public static SimpleMatrix process(ArrayList<IMUsensor> dataList, int m)
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
		
		SimpleMatrix dcm = new SimpleMatrix(Rotation.euler2dcm(new double[] {pitch,roll,yaw}));
		dcm = Rotation.reorthonormDcm(dcm);
		return dcm;
	}
	
	
}
