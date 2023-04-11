package com.ENGO623Final.Util;

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
	
	public static double getGravity(double lat,double alt) {
		double sin2lat = Math.pow(Math.sin(lat), 2); 
		double g = (a1*(1+(a2*sin2lat)+(a3*Math.pow(sin2lat, 2))))+((a4+(a5*sin2lat))*alt)+(a6*(Math.pow(alt,2)));
		return g;
	}

}
