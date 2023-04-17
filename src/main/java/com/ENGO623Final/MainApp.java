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
import com.ENGO623Final.helper.InitialAlignment;
import com.ENGO623Final.models.ImuSensor;
import com.ENGO623Final.models.State;

public class MainApp {

	public static void main(String args[]) {
		try {
			System.out.println("STARTED");
			File output = new File("C:\\Users\\naman.agarwal\\Documents\\GNSS\\ENGO-623\\project\\result.txt");
			PrintStream stream;
			stream = new PrintStream(output);
			System.setOut(stream);
			ArrayList<ImuSensor> dataList = Parser.getData("project_data.BIN");
			double[] llh0 = new double[] {Math.toRadians(51.07995352),Math.toRadians(-114.13371127),1118.502};
			// Perform Self-Alignment assuming for first 100 recordings, the IMU remains static/stationary
			SimpleMatrix dcm = InitialAlignment.process2(dataList, 67*120,llh0);
//			double[] euler = Rotation.dcm2euler(Matrix.matrix2Array(dcm));
//			SimpleMatrix q = Rotation.dcm2quaternion(dcm);
//			q = Rotation.reorthonormQuaternion(q);
//			SimpleMatrix dcm2 = Rotation.quaternion2dcm(q);
//			double[] euler2 = Rotation.dcm2euler(Matrix.matrix2Array(dcm2));
//			IntStream.range(0, 3).forEach(i->euler[i] = Math.toDegrees(euler[i]));
//			IntStream.range(0, 3).forEach(i->euler2[i] = Math.toDegrees(euler2[i]));
			System.out.println();
			TreeMap<Long,State> stateList = Mechanization.process(dataList, dcm, llh0);
			GraphPlotter.graphIMU(dataList);
			GraphPlotter.graphLLH(stateList);
			GraphPlotter.graphState(stateList, LatLonUtil.lla2ecef(llh0, false), Rotation.dcm2euler(Matrix.matrix2Array(dcm)));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
