package com.ENGO623Final.Util;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.net.URL;
import java.nio.file.Paths;
import java.util.ArrayList;

import com.ENGO623Final.models.IMUsensor;



public class Parser {

	public static ArrayList<IMUsensor> getData(String fileName) throws Exception {
		URL resource = Parser.class.getClassLoader().getResource(fileName);
		File file = Paths.get(resource.toURI()).toFile();
		DataInputStream in = new DataInputStream(new FileInputStream(file));
		ArrayList<IMUsensor> dataList = new ArrayList<IMUsensor>();
		while (in.available() > 0) {
			double[] data = new double[7];
			for (int i = 0; i < 7; i++) {
					long l = in.readLong();
					data[i] = Double.longBitsToDouble(Long.reverseBytes(l));
			}
			dataList.add(new IMUsensor(data));
		}
		return dataList;
	}
}
