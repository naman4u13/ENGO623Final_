package com.ENGO623Final.models;

import org.ejml.simple.SimpleMatrix;

public class State {
	private double[] p = new double[3];
	private double[] v = new double[3];
	private SimpleMatrix dcm;
	


	public State(double pX, double pY, double pZ, double vX, double vY, double vZ, SimpleMatrix dcm) {
		super();
		this.p[0] = pX;
		this.p[1] = pY;
		this.p[2] = pZ;
		this.v[0] = vX;
		this.v[1] = vY;
		this.v[2] = vZ;
		this.dcm = dcm;
		
		
	}
	
	public State(State state) {
		super();
		this.p[0] = state.getP()[0];
		this.p[1] = state.getP()[1];
		this.p[2] = state.getP()[2];
		this.v[0] = state.getV()[0];
		this.v[1] = state.getV()[1];
		this.v[2] = state.getV()[2];
		this.dcm = new SimpleMatrix(state.getDcm());
		
		
	}

	public double[] getP() {
		return p;
	}

	public void setP(double[] p) {
		this.p = new double[] {p[0],p[1],p[2]};
	}

	public double[] getV() {
		return v = new double[] {v[0],v[1],v[2]};
	}

	public void setV(double[] v) {
		this.v = v;
	}

	public SimpleMatrix getDcm() {
		return dcm;
	}

	public void setDcm(SimpleMatrix dcm) {
		this.dcm = new SimpleMatrix(dcm);
	}



	
}
