package com.ENGO623Final.Util;

public class Matrix {
	public static double[][] getSkewSymMat(double[] a) {
		return getSkewSymMat(a, false);
	}

	public static double[][] getSkewSymMat(double[] a, boolean isNeg) {
		// Make a deep copy, so that source array does not get impacted
		double[] b = new double[] { a[0], a[1], a[2] };
		if (isNeg) {
			for (int i = 0; i < 3; i++) {
				b[i] *= -1;
			}

		}
		return new double[][] { { 0, -b[2], b[1] }, { b[2], 0, -b[0] }, { -b[1], b[0], 0 } };
	}
}
