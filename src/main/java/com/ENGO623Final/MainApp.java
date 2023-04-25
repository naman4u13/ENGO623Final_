package com.ENGO623Final;

import java.io.File;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

import com.ENGO623Final.Util.GraphPlotter;
import com.ENGO623Final.Util.LatLonUtil;
import com.ENGO623Final.Util.Matrix;
import com.ENGO623Final.Util.Parser;
import com.ENGO623Final.Util.Rotation;
import com.ENGO623Final.constants.ImuParams;
import com.ENGO623Final.estimation.KalmanFilter;
import com.ENGO623Final.helper.InitialAlignment;
import com.ENGO623Final.models.ImuSensor;
import com.ENGO623Final.models.State;

public class MainApp {

	public static void main(String args[]) {
		try {
			System.out.println("STARTED");
			File output = new File("C:\\Users\\naman.agarwal\\Documents\\GNSS\\ENGO-623\\project\\result3.txt");
			PrintStream stream;
			stream = new PrintStream(output);
			System.setOut(stream);
			// Parse the data file
			ArrayList<ImuSensor> dataList = Parser.getData("project_data.BIN");
			// Observed IMU measurement Sample Rate (in Hz)
			int sampleRate = 67;
			// Initial position as provided in the assignment
			double[] llh0 = new double[] { Math.toRadians(51.07995352), Math.toRadians(-114.13371127), 1118.502 };
			// Deterministic/Residual Bias
			double acc_bias = ImuParams.acc_bias(llh0[0], llh0[2]);
			double gyro_bias = ImuParams.gyro_bias;

			//Removing Deterministic/Residual bias from the IMU data
			for (ImuSensor imuSensor : dataList) {
				double[] acc = imuSensor.getAcc();
				double[] gyro = imuSensor.getGyro();
				for (int j = 0; j < 3; j++) {
					acc[j] -= acc_bias;
					gyro[j] -= gyro_bias;
				}
				imuSensor.setAcc(acc);
				imuSensor.setGyro(gyro);

			}

			// Perform Self-Alignment assuming for first 65 seconds, the IMU remains
			// static/stationary
			SimpleMatrix dcm = InitialAlignment.process(dataList, sampleRate * 90, llh0);
			
			for (ImuSensor imuSensor : dataList) {
				double[] acc = imuSensor.getAcc();
				double[] gyro = imuSensor.getGyro();
				for (int j = 0; j < 3; j++) {
					acc[j] += acc_bias;
					gyro[j] += gyro_bias;
				}
				imuSensor.setAcc(acc);
				imuSensor.setGyro(gyro);

			}

			// Initiate the Mechanization module
			TreeMap<Long, State> stateList = KalmanFilter.process(dataList, dcm, llh0, sampleRate);
			//Mechanization.process(dataList, dcm, llh0);
			// Plot Results
			GraphPlotter.graphIMU(dataList);
			GraphPlotter.graphLLH(stateList);
			GraphPlotter.graphState(stateList, LatLonUtil.lla2ecef(llh0, false),
					Rotation.dcm2euler(Matrix.matrix2Array(dcm)));
			
			State lastSt = stateList.lastEntry().getValue();
			System.out.println("Final Latitude, Longitude and Altitude in degrees and meter");
			System.out.println(Math.toDegrees(lastSt.getP()[0])+"   "+Math.toDegrees(lastSt.getP()[1])+"   "+lastSt.getP()[2]);
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
