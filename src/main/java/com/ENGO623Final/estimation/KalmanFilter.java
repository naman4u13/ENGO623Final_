package com.ENGO623Final.estimation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;
import org.jblas.DoubleMatrix;
import org.jblas.MatrixFunctions;

import com.ENGO623Final.Util.LatLonUtil;
import com.ENGO623Final.Util.Matrix;
import com.ENGO623Final.Util.Rotation;
import com.ENGO623Final.constants.ImuParams;
import com.ENGO623Final.models.ImuSensor;
import com.ENGO623Final.models.State;

public class KalmanFilter {

	private static double posCov= 0.01;
	private static double velCov = 0.0001;

	public static TreeMap<Long, State> process(ArrayList<ImuSensor> dataList, SimpleMatrix dcm0, double[] llh0,
			int sampleRate) throws Exception {
		double acc_bias = ImuParams.acc_bias(llh0[0], llh0[2]);
		double acc_bias_instability = ImuParams.acc_bias_instability(llh0[0], llh0[2]);
		double[] ecef0 = LatLonUtil.lla2ecef(llh0, false);
		State X = new State(llh0[0], llh0[1], llh0[2], 0, 0, 0, dcm0, acc_bias, acc_bias, acc_bias, ImuParams.gyro_bias,
				ImuParams.gyro_bias, ImuParams.gyro_bias);
		double attCov = Math.pow(Math.toRadians(5), 2);
		// Initial state covariance for Acc Bias is assumed to be 10 micro-g, where g = 10ms^-2
		double accBiasCov = Math.pow(10*1e-6*10, 2);
		// Initial state covariance for Gyro Bias is assumed to be 1 deg/hr
		double gyroBiasCov = Math.pow(10*ImuParams.degPerHr_2_radPerS, 2);
		double[] p0 = new double[] { posCov,posCov,posCov, velCov, velCov,velCov,attCov, attCov, attCov, accBiasCov, accBiasCov,
				accBiasCov, gyroBiasCov, gyroBiasCov, gyroBiasCov };
	
		SimpleMatrix P = new SimpleMatrix(15, 15);
		for (int i = 0; i < 15; i++) {
			P.set(i, i, p0[i]);
		}
		// PSD of random walk - N
		double acc_SN = Math.pow(ImuParams.VRW, 2);
		double gyro_SN = Math.pow(ImuParams.ARW, 2);
		// PSD of Bias Instability - B
		double acc_SB = 2 * Math.pow(acc_bias_instability, 2) * Math.log(2)
				/ (Math.PI * Math.pow(0.4365, 2) * ImuParams.acc_corr_time);
		double gyro_SB = 2 * Math.pow(ImuParams.gyro_bias_instability, 2) * Math.log(2)
				/ (Math.PI * Math.pow(0.4365, 2) * ImuParams.gyro_corr_time);
		double[] q = new double[] { acc_SN, acc_SN, acc_SN, gyro_SN, gyro_SN, gyro_SN, acc_SB, acc_SB, acc_SB, gyro_SB,
				gyro_SB, gyro_SB };
		int n = dataList.size();
		double prevTime = dataList.get(0).getTime();
		TreeMap<Long, State> stateList = new TreeMap<Long, State>();
		int count = 0;
		for (int i = 1; i < n; i++) {
			ImuSensor imuSensor = dataList.get(i);
			double time = imuSensor.getTime();
			double tau = time - prevTime;
			// Total State Prediction or Mechanization
			double[][] estInsObs = predictTotalState(X, imuSensor, tau);
			SimpleMatrix[] discParam = getDiscreteParams(X, estInsObs[0], estInsObs[1], tau, q);
			P = predictErrorState(X, P, discParam[0], discParam[1]);
			
			if (i % (sampleRate*1) == 0) {
				P = update(X, P, ecef0);
				count++;
			}
			prevTime = time;
			stateList.put((long) (time * 1e3), new State(X));
		}
		System.err.println("UPDATES COUNT: "+ count);
		return stateList;
	}

	// NED Mechanization
	private static double[][] predictTotalState(State X, ImuSensor imuSensor, double tau) throws Exception {

		// Update Bias state
		double[] accBias = Arrays.stream(X.getAccBias()).map(i -> i * Math.exp(-tau / ImuParams.acc_corr_time))
				.toArray();
		double[] gyroBias = Arrays.stream(X.getGyroBias()).map(i -> i * Math.exp(-tau / ImuParams.gyro_corr_time))
				.toArray();
		double[] obsAcc = imuSensor.getAcc();
		double[] estAcc = IntStream.range(0, 3).mapToDouble(j -> obsAcc[j] - (accBias[j])).toArray();
		double[] obsGyro = imuSensor.getGyro();
		double[] estGyro = IntStream.range(0, 3).mapToDouble(j -> obsGyro[j] - (gyroBias[j])).toArray();
		double lat = X.getP()[0];
		double lon = X.getP()[1];
		double alt = X.getP()[2];
		double[] vel = X.getV();
		double Rn = LatLonUtil.getNormalEarthRadius(lat);
		double Rm = LatLonUtil.getMeridianEarthRadius(lat);
		double earthAngularRate = LatLonUtil.omega_ie;

		// Attitude update
		final SimpleMatrix oldDcm = new SimpleMatrix(X.getDcm());
		SimpleMatrix omega_b_ib = new SimpleMatrix(Matrix.getSkewSymMat(estGyro));
		double[] _omega_n_ie = new double[] { earthAngularRate * Math.cos(lat), 0, -earthAngularRate * Math.sin(lat) };
		SimpleMatrix omega_n_ie = new SimpleMatrix(Matrix.getSkewSymMat(_omega_n_ie));
		double[] _omega_n_en = new double[] { vel[1] / (Rn + alt), -vel[0] / (Rm + alt),
				-vel[1] * Math.tan(lat) / (Rn + alt) };
		SimpleMatrix omega_n_en = new SimpleMatrix(Matrix.getSkewSymMat(_omega_n_en));
		SimpleMatrix I = SimpleMatrix.identity(3);
		SimpleMatrix newDcm = (oldDcm.mult(I.plus(omega_b_ib.scale(tau))))
				.minus((omega_n_ie.plus(omega_n_en)).mult(oldDcm.scale(tau)));
		newDcm = new SimpleMatrix(Rotation.reorthonormDcm(newDcm));

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

		X.setDcm(newDcm);
		X.setV(new double[] { newVel.get(0), newVel.get(1), newVel.get(2) });
		X.setP(new double[] { newLat, newLon, newAlt });
		return new double[][] { estAcc, estGyro };

	}

	private static SimpleMatrix[] getDiscreteParams(State X, double[] estAcc, double[] estGyro, double tau,
			double[] q) {

		double lat = X.getP()[0];
		double lon = X.getP()[1];
		double alt = X.getP()[2];
		double vn = X.getV()[0];
		double ve = X.getV()[1];
		double vd = X.getV()[2];
		SimpleMatrix dcm = new SimpleMatrix(X.getDcm());
		double Rn = LatLonUtil.getNormalEarthRadius(lat);
		double Rm = LatLonUtil.getMeridianEarthRadius(lat);
		double omega_ie = LatLonUtil.omega_ie;
		double slat = Math.sin(lat);
		double clat = Math.cos(lat);
		double tlat = Math.tan(lat);
		// Instead of Re has used original denominator in rhos
		double rhoE = -vn / (Rm + alt);
		double rhoN = ve / (Rn + alt);
		double rhoD = -ve * tlat / (Rn + alt);
		double OmegaN = omega_ie * clat;
		double OmegaD = -omega_ie * slat;
		double omega_N = OmegaN + rhoN;
		double omega_E = rhoE;
		double omega_D = OmegaD + rhoD;
		// Unsure whether to use Rn+alt or Rm+alt for Re
		double Re = Rn + alt;
		double kD = vd / Re;
		double g = LatLonUtil.getGravity(lat, alt);
		double F63 = Math.pow(rhoN, 2) + Math.pow(rhoE, 2) - (2 * g / Re);

		SimpleMatrix Fpp = new SimpleMatrix(new double[][] { { 0, 0, -vn / Math.pow(Rm + alt, 2) },
				{ ve * slat / ((Rn + alt) * Math.pow(clat, 2)), 0, -ve / (Math.pow(Rn + alt, 2) * clat) },
				{ 0, 0, 0 } });
		SimpleMatrix Fpv = new SimpleMatrix(
				new double[][] { { 1 / (Rm + alt), 0, 0 }, { 0, 1 / ((Rn + alt) * clat), 0 }, { 0, 0, -1 } });
		SimpleMatrix Fpj = new SimpleMatrix(new double[3][3]);
		SimpleMatrix Fjp = new SimpleMatrix(new double[][] { { omega_ie * slat, 0, ve / Math.pow(Rn + alt, 2) },
				{ 0, 0, -vn / Math.pow(Rm + alt, 2) }, { (omega_ie * clat) + (ve / ((Rn + alt) * Math.pow(clat, 2))), 0,
						-(ve * tlat) / Math.pow(Rn + alt, 2) } });
		SimpleMatrix Fjv = new SimpleMatrix(
				new double[][] { { 0, -1 / (Rn + alt), 0 }, { 1 / (Rm + alt), 0, 0 }, { 0, tlat / (Rn + alt), 0 } });
		SimpleMatrix Fjj = new SimpleMatrix(Matrix.getSkewSymMat(new double[] { -omega_N, -omega_E, -omega_D }));
		SimpleMatrix Fvp = new SimpleMatrix(new double[][] {
				{ (-2 * OmegaN * ve) - (rhoN * ve / Math.pow(clat, 2)), 0, (rhoE * kD) - (rhoN * rhoD) },
				{ (2 * ((OmegaN * vn) + (OmegaD * vd))) + (rhoN * vn / Math.pow(clat, 2)), 0,
						(-rhoE * rhoD) - (kD * rhoN) },
				{ -2 * ve * OmegaD, 0, F63 } });
		SimpleMatrix Fvv = new SimpleMatrix(new double[][] { { kD, 2 * omega_D, -rhoE },
				{ -(omega_D + OmegaD), kD - (rhoE * tlat), omega_N + OmegaN }, { 2 * rhoE, -2 * omega_N, 0 } });
		SimpleMatrix Fvj = new SimpleMatrix(Matrix.getSkewSymMat(estAcc, true));
		SimpleMatrix zero = new SimpleMatrix(3, 3);
		SimpleMatrix identity = SimpleMatrix.identity(3);
		SimpleMatrix Fva = identity;
		SimpleMatrix Fjg = identity;
		SimpleMatrix Faa = identity.scale(-1 / ImuParams.acc_corr_time);
		SimpleMatrix Fgg = identity.scale(-1 / ImuParams.gyro_corr_time);
		SimpleMatrix F = (Fpp.concatColumns(Fpv, Fpj, zero, zero))
				.concatRows(Fvp.concatColumns(Fvv, Fvj, dcm.scale(-1).mult(Fva), zero))
				.concatRows(Fjp.concatColumns(Fjv, Fjj, zero, dcm.mult(Fjg)))
				.concatRows(zero.concatColumns(zero, zero, Faa, zero))
				.concatRows(zero.concatColumns(zero, zero, zero, Fgg));

		// First order approx of phi
		SimpleMatrix phi = SimpleMatrix.identity(15).plus(F.scale(tau));
		SimpleMatrix G = (zero.concatColumns(zero, zero, zero))
				.concatRows(dcm.scale(-1).concatColumns(zero, zero, zero))
				.concatRows(zero.concatColumns(dcm, zero, zero)).concatRows(zero.concatColumns(zero, identity, zero))
				.concatRows(zero.concatColumns(zero, zero, identity));

		SimpleMatrix Q = new SimpleMatrix(12, 12);
		IntStream.range(0, 12).forEach(i -> Q.set(i, i, q[i]));
		
		
		SimpleMatrix __A = ((F.scale(-1).concatColumns(G.mult(Q).mult(G.transpose()))).concatRows(new SimpleMatrix(15,15).concatColumns(F.transpose()))).scale(tau);
		double[][] _A = Matrix.matrix2Array(__A);
		DoubleMatrix A = new DoubleMatrix(_A);
		DoubleMatrix _B = MatrixFunctions.expm(A);
		SimpleMatrix B = new SimpleMatrix(Matrix.matrix2Array(_B));
		SimpleMatrix phi2 = B.extractMatrix(B.numRows()-15, B.numRows(), B.numCols()-15, B.numCols()).transpose();
		SimpleMatrix Qk2 = phi2.mult(B.extractMatrix(0, 15, B.numCols()-15, B.numCols()));
		
		
		// First order approx of Qk
		SimpleMatrix Qk = G.mult(Q).mult(G.transpose()).scale(tau);
		return new SimpleMatrix[] { phi2, Qk2 };

	}

	private static SimpleMatrix predictErrorState(State X, SimpleMatrix P, SimpleMatrix phi, SimpleMatrix Qk) {
		P = (phi.mult(P).mult(phi.transpose())).plus(Qk);
		return P;
	}

	public static SimpleMatrix update(State X, SimpleMatrix P, double[] ecef0) throws Exception {

		double[] llh = X.getP();
		double[] vel = X.getV();
		SimpleMatrix dcm = X.getDcm();
		double[] ecef = LatLonUtil.lla2ecef(llh, false);

		SimpleMatrix H = null;
		SimpleMatrix Z = null;
		SimpleMatrix R = null;

		double[] z = new double[] { ecef0[0] - ecef[0], ecef0[1] - ecef[1], ecef0[2] - ecef[2], 0 - vel[0], 0 - vel[1],
				0 - vel[2] };
		R = new SimpleMatrix(6, 6);
		for (int i = 0; i < 3; i++) {
			R.set(i, i, posCov);
			R.set(i + 3, i + 3, velCov);
		}
		H = new SimpleMatrix(6, 15);
		for (int i = 0; i < 6; i++) {
			H.set(i, i, 1);
		}
		Z = new SimpleMatrix(z.length, 1, true, z);
		SimpleMatrix Ht = H.transpose();
		SimpleMatrix K = null;
		// Kalman Gain

		K = P.mult(Ht).mult(((H.mult(P).mult(Ht)).plus(R)).invert());

		// Posterior State Estimate
		// As prior deltaX is zero, there is no 'ze'
		SimpleMatrix deltaX = K.mult(Z);
		SimpleMatrix KH = K.mult(H);
		SimpleMatrix I = SimpleMatrix.identity(KH.numRows());
		// Posterior Estimate Error Joseph Form to ensure Positive Definiteness P =
		// (I-KH)P(I-KH)' + KRK'
		P = ((I.minus(KH)).mult(P).mult((I.minus(KH)).transpose())).plus(K.mult(R).mult(K.transpose()));

		// Update Total State
		ecef[0] += deltaX.get(0);
		ecef[1] += deltaX.get(1);
		ecef[2] += deltaX.get(2);
		double[] _llh = LatLonUtil.ecef2lla(ecef, false);
		X.setP(_llh);
		vel[0] += deltaX.get(3);
		vel[1] += deltaX.get(4);
		vel[2] += deltaX.get(5);
		X.setV(LatLonUtil.ecef2ned(vel, ecef, false));
		SimpleMatrix updateDcm = new SimpleMatrix(
				Matrix.getSkewSymMat(new double[] { deltaX.get(6), deltaX.get(7), deltaX.get(8) }));
		double[] updateEuler = new double[] {deltaX.get(6), deltaX.get(7), deltaX.get(8)};
		SimpleMatrix updateDcm2 = new SimpleMatrix(Rotation.euler2dcm(updateEuler));
		updateDcm2 = Rotation.reorthonormDcm(updateDcm2);
		//dcm = (SimpleMatrix.identity(3).minus(updateDcm)).mult(dcm);
		dcm = updateDcm2.mult(dcm);
		
		dcm = Rotation.reorthonormDcm(dcm);
		X.setDcm(dcm);
		X.setAccBias(IntStream.range(0, 3).mapToDouble(i -> X.getAccBias()[i] + deltaX.get(9 + i)).toArray());
		X.setGyroBias(IntStream.range(0, 3).mapToDouble(i -> X.getGyroBias()[i] + deltaX.get(12 + i)).toArray());
		return P;
	}

}
