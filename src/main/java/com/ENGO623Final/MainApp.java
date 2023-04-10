package com.ENGO623Final;

import java.io.File;
import java.io.PrintStream;
import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

import com.ENGO623Final.Util.Parser;
import com.ENGO623Final.helper.InitialAlignment;
import com.ENGO623Final.models.IMUsensor;

public class MainApp {

	public static void main(String args[]) {
		try {
			System.out.println("STARTED");
			File output = new File("C:\\Users\\naman.agarwal\\Documents\\GNSS\\ENGO-623\\project\\result.txt");
			PrintStream stream;
			stream = new PrintStream(output);
			System.setOut(stream);
			ArrayList<IMUsensor> dataList = Parser.getData("project_data.BIN");
			// Assuming for first 100 recordings, the IMU remains static/stationary 
			SimpleMatrix dcm = InitialAlignment.process(dataList, 100);
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
