package com.ENGO623Final.Util;

import java.util.ArrayList;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.ui.ApplicationFrame;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.RefineryUtilities;

import com.ENGO623Final.constants.ImuParams;
import com.ENGO623Final.models.ImuSensor;
import com.ENGO623Final.models.State;

public class GraphPlotter extends ApplicationFrame {

	public GraphPlotter(ArrayList<ImuSensor> dataList, boolean isAcc, String name) {

		super(name);
		String str = isAcc ? "(m/s^2)" : "(rad/s)";
		JFreeChart chart = ChartFactory.createXYLineChart(name + str + " vs Time(s)", "Time(s)", name + str,
				createDataSetImu(dataList, isAcc));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(ArrayList<Double> dataList, ArrayList<Double> timeList, String name, String title) {

		super(title);

		JFreeChart chart = ChartFactory.createXYLineChart(name + " vs Time(s)", "Time(s)", name,
				createDataSetLLH(dataList, timeList, name));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	private XYDataset createDataSetImu(ArrayList<ImuSensor> dataList, boolean isAcc) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		final XYSeries x = new XYSeries("X");
		final XYSeries y = new XYSeries("Y");
		final XYSeries z = new XYSeries("Z");
		for (ImuSensor imu : dataList) {
			double[] data = isAcc ? imu.getAcc() : imu.getGyro();
			int time = (int) (imu.getTime() * 1e3);
			x.add(time, data[0]);
			y.add(time, data[1]);
			z.add(time, data[2]);
		}
		dataset.addSeries(x);
		dataset.addSeries(y);
		dataset.addSeries(z);
		return dataset;
	}

	private XYDataset createDataSetLLH(ArrayList<Double> dataList, ArrayList<Double> timeList, String name) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		final XYSeries series = new XYSeries(name);
		int n = timeList.size();
		for (int i = 0; i < n; i++) {
			series.add(timeList.get(i), dataList.get(i));
		}

		dataset.addSeries(series);
		return dataset;
	}

	public static void graphIMU(ArrayList<ImuSensor> dataList) {

		GraphPlotter chart = new GraphPlotter(dataList, true, "Accelerometer");
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);

		chart = new GraphPlotter(dataList, false, "Gyroscope");
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);
	}

	public static void graphLLH(TreeMap<Long, State> stateMap) {

		ArrayList<Double> timeList = new ArrayList<Double>();
		ArrayList<Double>[] llhList = new ArrayList[3];
		IntStream.range(0, 3).forEach(i -> llhList[i] = new ArrayList<Double>());
		for (long t : stateMap.keySet()) {
			timeList.add(t*1.0/1000);
			double[] llh = stateMap.get(t).getP();
			llhList[0].add(Math.toDegrees(llh[0]));
			llhList[1].add(Math.toDegrees(llh[1]));
			llhList[2].add(llh[2]);
		}
		String[] names = new String[] { "Latitiude(in deg)", "Longitude(in deg)", "Altitude(in meter)" };
		for (int i = 0; i < 3; i++) {
			GraphPlotter chart = new GraphPlotter(llhList[i], timeList, names[i], names[i]);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}

	}

	public static void graphState(TreeMap<Long, State> stateMap, double[] ecef0, double[] euler0) {

		ArrayList<Double> timeList = new ArrayList<Double>();
		ArrayList<Double>[] posErrList = new ArrayList[3];
		ArrayList<Double>[] velErrList = new ArrayList[3];
		ArrayList<Double>[] attErrList = new ArrayList[3];
		for (int i = 0; i < 3; i++) {
			posErrList[i] = new ArrayList<Double>();
			velErrList[i] = new ArrayList<Double>();
			attErrList[i] = new ArrayList<Double>();
		}
		double g = LatLonUtil.getGravity(Math.toRadians(51.07995352),1118.502);
		for (long t : stateMap.keySet()) {
			timeList.add(t*1.0/1000);
			State state = stateMap.get(t);
			double[] llh = state.getP();
			double[] vel = state.getV();
			double[] att_euler = Rotation.dcm2euler(Matrix.matrix2Array(state.getDcm()));
			
		
			double[] ecef = LatLonUtil.lla2ecef(llh, false);
			double[] posErr = LatLonUtil.ecef2enu(ecef, ecef0, true);
			double[] velErr = new double[] { vel[1], vel[0], -vel[2] };
			double[] attErr = IntStream.range(0, 3).mapToDouble(i -> Math.toDegrees(att_euler[i] - euler0[i])*60).toArray();
			for (int i = 0; i < 3; i++) {
				posErrList[i].add(posErr[i]);
				velErrList[i].add(velErr[i]);
				attErrList[i].add(attErr[i]);
			}
		

		}
		String[] names = new String[] { "E(in m)", "N(in m)", "U(in m)" };
		for (int i = 0; i < 3; i++) {
			GraphPlotter chart = new GraphPlotter(posErrList[i], timeList, "Position Error "+names[i], "Position Error "+names[i]);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}
		names = new String[] { "E(in m/s)", "N(in m/s)", "U(in m/s)" };
		for (int i = 0; i < 3; i++) {
			GraphPlotter chart = new GraphPlotter(velErrList[i], timeList, "Velocity Error "+names[i], "Velocity Error "+names[i]);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}
		names = new String[] { "Roll(in arcmin)", "Pitch(in arcmin)", "Yaw(in arcmin)" };
		for (int i = 0; i < 3; i++) {
			GraphPlotter chart = new GraphPlotter(attErrList[i], timeList, names[i], names[i]);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}
		
		System.out.println("Final Position Error in ENU");
		System.out.println(posErrList[0].get(posErrList[0].size()-1)+"   "+posErrList[1].get(posErrList[1].size()-1)+"   "+posErrList[2].get(posErrList[2].size()-1));
		System.out.println("Final Velocity Error in ENU");
		System.out.println(velErrList[0].get(velErrList[0].size()-1)+"   "+velErrList[1].get(velErrList[1].size()-1)+"   "+velErrList[2].get(velErrList[2].size()-1));
		System.out.println("Final Attitude Error in arcmin");
		System.out.println(attErrList[0].get(attErrList[0].size()-1)+"   "+attErrList[1].get(attErrList[1].size()-1)+"   "+attErrList[2].get(attErrList[2].size()-1));
	}
}
