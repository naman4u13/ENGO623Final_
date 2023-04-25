package com.ENGO623Final.Util;

import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

public class LatLonUtil {
	// All are WGS-84 params
	// Semi-major axis or Equatorial radius
	private static final double a = 6378137;
	// flattening
	private static final double f = 1 / 298.257223563;
	// Semi-minor axis or Polar radius
	private static final double b = 6356752.314245;
	private static final double e = Math.sqrt((Math.pow(a, 2) - Math.pow(b, 2)) / Math.pow(a, 2));
	private static final double e2 = Math.sqrt((Math.pow(a, 2) - Math.pow(b, 2)) / Math.pow(b, 2));
	// Earth Angular rate in rad/s
	public static final double omega_ie = 7.292115147e-5;
	// Earth’s gravitational constant in m^3/s^2
	private static final double mu = 3.986004418e14;

	// Parameters for gravity estimation
	private static double a1 = 9.7803267715;
	private static double a2 = 0.0052790414;
	private static double a3 = 0.0000232718;
	private static double a4 = -0.000003087691089;
	private static double a5 = 0.000000004397731;
	private static double a6 = 0.000000000000721;

	/*
	 * The radius of curvature for east-west motion is known as the transverse
	 * radius of curvature, Re, it is also known as normal radius or prime vertical
	 * radius
	 */
	public static double getNormalEarthRadius(double lat) {
		double Rn = a / Math.sqrt(1 - Math.pow(e * Math.sin(lat), 2));
		return Rn;
	}

	/*
	 * The radius of curvature for north-south motion is known as the meridian
	 * radius
	 */
	public static double getMeridianEarthRadius(double lat) {
		double Rm = (a * (1 - Math.pow(e, 2))) / Math.pow(1 - Math.pow(e * Math.sin(lat), 2), 1.5);
		return Rm;
	}

	public static double getGravity(double lat, double alt) {
		double sin2lat = Math.pow(Math.sin(lat), 2);
		double g = (a1 * (1 + (a2 * sin2lat) + (a3 * Math.pow(sin2lat, 2)))) + ((a4 + (a5 * sin2lat)) * alt)
				+ (a6 * (Math.pow(alt, 2)));
		return g;
	}

	public static double getGravity2(double lat, double alt) {
		double g0 = (9.7803253359 * (1 + (0.001931853 * Math.pow(Math.sin(lat), 2))))
				/ Math.sqrt(1 - Math.pow(e * Math.sin(lat), 2));
		double g = g0
				* (1 - ((2 * alt / a) * (1 + f + (Math.pow(omega_ie * a, 2) * b / mu))) + (3 * Math.pow(alt / a, 2)));
		return g;
	}

	public static double[] lla2ecef(double[] lla, boolean isDegree) {

		double finv = 1 / f;
		double esq = (2 - 1 / finv) / finv;
		double dtr = 1;
		if (isDegree) {
			dtr = Math.PI / 180;
		}

		double lat = lla[0] * dtr; // rad
		double lon = lla[1] * dtr; // rad
		double alt = lla[2]; // m

		double N = a / Math.sqrt(1 - esq * Math.pow(Math.sin(lat), 2));

		double x = (N + alt) * Math.cos(lat) * Math.cos(lon);
		double y = (N + alt) * Math.cos(lat) * Math.sin(lon);
		double z = ((1 - esq) * N + alt) * Math.sin(lat);

		double[] ecef = { x, y, z }; // m
		return ecef;
	}

	public static double[] ecef2lla(double[] ECEF) {

		return ecef2lla(ECEF, true);

	}

	public static double[] ecef2lla(double[] ECEF, boolean inDeg) {

		double x = ECEF[0];
		double y = ECEF[1];
		double z = ECEF[2];
		double[] lla = { 0, 0, 0 };
		double lat = 0, lon, height, N, theta, p;

		p = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

		theta = Math.atan((z * a) / (p * b));

		lon = Math.atan(y / x);

		if (x < 0) {
			if (y > 0) {
				lon = Math.PI + lon;
			} else {
				lon = -Math.PI + lon;
			}
		}
		for (int i = 0; i < 3; i++) {
			lat = Math.atan(((z + Math.pow(e2, 2) * b * Math.pow(Math.sin(theta), 3))
					/ ((p - Math.pow(e, 2) * a * Math.pow(Math.cos(theta), 3)))));
			theta = Math.atan((Math.tan(lat) * b) / (a));

		}

		N = a / (Math.sqrt(1 - (Math.pow(e, 2) * Math.pow(Math.sin(lat), 2))));

		height = (p * Math.cos(lat)) + ((z + (Math.pow(e, 2) * N * Math.sin(lat))) * Math.sin(lat)) - N;
		if (inDeg) {
			lon = lon * 180 / Math.PI;
			lat = lat * 180 / Math.PI;
		}
		lla[0] = lat;
		lla[1] = lon;
		lla[2] = height;
		return lla;
	}

	public static double[] ecef2ned(double[] ecef, double[] refEcef, boolean isPos) {
		double[] ned = enu_ned_convert(ecef2enu(ecef, refEcef, isPos));
		return ned;
	}

	// Reference -
	// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
	public static double[] ecef2enu(double[] ecef, double[] refEcef, boolean isPos) {
		double[] _diff = new double[] { ecef[0], ecef[1], ecef[2] };
		if (isPos) {
			IntStream.range(0, 3).forEach(i -> _diff[i] = _diff[i] - refEcef[i]);
		}
		SimpleMatrix diff = new SimpleMatrix(3, 1, false, _diff);
		SimpleMatrix R = new SimpleMatrix(getEcef2EnuRotMat(refEcef));
		SimpleMatrix _enu = R.mult(diff);
		double[] enu = new double[] { _enu.get(0), _enu.get(1), _enu.get(2) };
		return enu;
	}

	public static double[][] getEcef2EnuRotMat(double[] refEcef) {
		double[] llh = ecef2lla(refEcef);
		double lat = Math.toRadians(llh[0]);
		double lon = Math.toRadians(llh[1]);
		double[][] R = new double[][] { { -Math.sin(lon), Math.cos(lon), 0 },
				{ -Math.sin(lat) * Math.cos(lon), -Math.sin(lat) * Math.sin(lon), Math.cos(lat) },
				{ Math.cos(lat) * Math.cos(lon), Math.cos(lat) * Math.sin(lon), Math.sin(lat) } };
		return R;
	}

	public static double[] enu_ned_convert(double[] x) {
		SimpleMatrix Y = enu_ned_convert(new SimpleMatrix(3, 1, true, x));
		double[] y = new double[] { Y.get(0), Y.get(1), Y.get(2) };
		return y;
	}

	public static double[][] enu_ned_convert(double[][] x) {
		SimpleMatrix Y = enu_ned_convert(new SimpleMatrix(x));
		double[][] y = Matrix.matrix2Array(Y);
		return y;
	}

	// DCM for ENU to NED and vice versa is same
	public static SimpleMatrix enu_ned_convert(SimpleMatrix x) {

		double[][] c = new double[][] { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } };
		SimpleMatrix C = new SimpleMatrix(c);
		SimpleMatrix y = C.mult(x);
		return y;

	}
}
