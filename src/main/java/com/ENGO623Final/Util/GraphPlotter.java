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

import com.ENGO623Final.models.ImuSensor;
import com.ENGO623Final.models.State;

public class GraphPlotter extends ApplicationFrame {

	public GraphPlotter(ArrayList<ImuSensor> dataList, boolean isAcc, String name) {

		super(name);
		String str = isAcc ? "(m/s^2)" : "(rad/s)";
		JFreeChart chart = ChartFactory.createXYLineChart(name + str + " vs Time(ms)", "Time(ms)", name + str,
				createDataSetImu(dataList, isAcc));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(ArrayList<Double> dataList, ArrayList<Long> timeList, String name) {

		super(name);

		JFreeChart chart = ChartFactory.createXYLineChart(name + " vs Time(ms)", "Time(ms)", name,
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

	private XYDataset createDataSetLLH(ArrayList<Double> dataList, ArrayList<Long> timeList, String name) {
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
		
		ArrayList<Long> timeList = new ArrayList<Long>();
		ArrayList<Double>[] llhList = new ArrayList[3];
		IntStream.range(0, 3).forEach(i -> llhList[i] = new ArrayList<Double>());
		for (long t : stateMap.keySet()) {
			timeList.add(t);
			double[] llh = stateMap.get(t).getP();
			llhList[0].add(Math.toDegrees(llh[0]));
			llhList[1].add(Math.toDegrees(llh[1]));
			llhList[2].add(llh[2]);
		}
		String[] names = new String[] { "Latitiude(in deg)", "Longitude(in deg)", "Altiude(in meter)" };
		for (int i = 0; i < 3; i++) {
			GraphPlotter chart = new GraphPlotter(llhList[i], timeList, names[i]);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}

	}
}
