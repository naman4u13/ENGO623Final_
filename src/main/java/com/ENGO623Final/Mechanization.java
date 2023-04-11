package com.ENGO623Final;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

import com.ENGO623Final.Util.LatLonUtil;
import com.ENGO623Final.Util.Matrix;
import com.ENGO623Final.Util.Rotation;
import com.ENGO623Final.constants.ImuParams;
import com.ENGO623Final.models.ImuSensor;
import com.ENGO623Final.models.State;

 

public class Mechanization {
	
	 
	public static TreeMap<Long,State> process(ArrayList<ImuSensor> dataList,SimpleMatrix dcm,double[] llh0)
	{
		State X = new State(llh0[0], llh0[1], llh0[2], 0,0,0, dcm, ImuParams.acc_bias, ImuParams.acc_bias, ImuParams.acc_bias, ImuParams.gyro_bias, ImuParams.gyro_bias, ImuParams.gyro_bias);
		int n = dataList.size();
		double prevTime = dataList.get(0).getTime();
		TreeMap<Long,State> stateList = new TreeMap<Long,State>();
		for(int i=1;i<n;i++)
		{
			ImuSensor imuSensor = dataList.get(i);
			double time = imuSensor.getTime();
			double tau = time-prevTime;
			predictTotalState(X, imuSensor, tau);
			prevTime = time;
			stateList.put((long)(time*1e3),new State(X));
			
		}
		return stateList;
	}
	
	private static double[][] predictTotalState(State X,ImuSensor imuSensor, double tau) {

		// Update Bias state
		double[] accBias = Arrays.stream(X.getAccBias())
				.map(i -> i * Math.exp(-tau / ImuParams.acc_corr_time)).toArray();
		double[] gyroBias = Arrays.stream(X.getGyroBias())
				.map(i -> i * Math.exp(-tau / ImuParams.gyro_corr_time)).toArray();

		double[] obsAcc = imuSensor.getAcc();
		double[] estAcc = IntStream.range(0, 3).mapToDouble(j -> obsAcc[j] - accBias[j]).toArray();
		double[] obsGyro = imuSensor.getGyro();
		double[] estGyro = IntStream.range(0, 3).mapToDouble(j -> obsGyro[j] - gyroBias[j]).toArray();

		double lat = X.getP()[0];
		double lon = X.getP()[1];
		double alt = X.getP()[2];
		double[] vel = X.getV();
		double Rn = LatLonUtil.getNormalEarthRadius(lat);
		double Rm = LatLonUtil.getMeridianEarthRadius(lat);
		double earthAngularRate = LatLonUtil.omega_ie;

		// Attitude update
		SimpleMatrix oldDcm = new SimpleMatrix(X.getDcm());
		SimpleMatrix omega_b_ib = new SimpleMatrix(Matrix.getSkewSymMat(estGyro));
		double[] _omega_n_ie = new double[] { earthAngularRate * Math.cos(lat), 0, -earthAngularRate * Math.sin(lat) };
		SimpleMatrix omega_n_ie = new SimpleMatrix(Matrix.getSkewSymMat(_omega_n_ie));
		double[] _omega_n_en = new double[] { vel[1] / (Rn + alt), -vel[0] / (Rm + alt),
				-vel[1] * Math.tan(lat) / (Rn + alt) };
		SimpleMatrix omega_n_en = new SimpleMatrix(Matrix.getSkewSymMat(_omega_n_en));
		SimpleMatrix I = SimpleMatrix.identity(3);
		SimpleMatrix newDcm = (oldDcm.mult(I.plus(omega_b_ib.scale(tau))))
				.minus((omega_n_ie.plus(omega_n_en)).mult(oldDcm.scale(tau)));
		newDcm = Rotation.reorthonormDcm(newDcm);

		// Specific-Force Frame Transformation
		SimpleMatrix f_b_ib = new SimpleMatrix(3, 1, true, estAcc);
		SimpleMatrix f_n_ib = (oldDcm.plus(newDcm)).scale(0.5).mult(f_b_ib);

		// Velocity Update
		SimpleMatrix oldVel = new SimpleMatrix(3, 1, true, vel);
		SimpleMatrix g_n_b = new SimpleMatrix(3, 1, true, new double[] { 0, 0, LatLonUtil.getGravity(lat, alt) });
		SimpleMatrix newVel = oldVel
				.plus((f_n_ib.plus(g_n_b).minus((omega_n_en.plus((omega_n_ie.scale(2)))).mult(oldVel))).scale(tau));

		// Position Update
		double newAlt = alt - ((tau / 2) * (oldVel.get(2) + newVel.get(2)));
		double newLat = lat + ((tau / 2) * ((oldVel.get(0) / (Rm + alt)) + (newVel.get(0) / (Rm + newAlt))));
		double newRn = LatLonUtil.getNormalEarthRadius(newLat);
		double newLon = lon + ((tau / 2) * ((oldVel.get(1) / ((Rn + alt) * Math.cos(lat)))
				+ (newVel.get(1) / ((newRn + newAlt) * Math.cos(newLat)))));

		

		X.setAccBias(accBias);
		X.setGyroBias(gyroBias);
		X.setDcm(newDcm);
		X.setV(new double[] { newVel.get(0), newVel.get(1), newVel.get(2) });
		X.setP(new double[] { newLat, newLon, newAlt });
	

		return new double[][] { estAcc, estGyro };
	}
	
}
