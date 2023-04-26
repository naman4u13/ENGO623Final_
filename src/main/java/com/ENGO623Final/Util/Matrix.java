package com.ENGO623Final.Util;

import org.ejml.simple.SimpleMatrix;
import org.jblas.DoubleMatrix;

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
	
	public static double[][] matrix2Array(SimpleMatrix matrix) {
		double[][] array = new double[matrix.numRows()][matrix.numCols()];
		for (int r = 0; r < matrix.numRows(); r++) {
			for (int c = 0; c < matrix.numCols(); c++) {
				array[r][c] = matrix.get(r, c);
			}
		}
		return array;
	}
	
	public static double[][] matrix2Array(DoubleMatrix matrix) {
		double[][] array = new double[matrix.rows][matrix.columns];
		for (int r = 0; r < matrix.rows; r++) {
			for (int c = 0; c < matrix.columns; c++) {
				array[r][c] = matrix.get(r, c);
			}
		}
		return array;
	}
	
	public static double[][] check(double[][] mat)
	{
		int r = mat.length;
		int c = mat[0].length;
		double[][] new_mat = new double[r][c];
		for(int i=0;i<r;i++)
		{
			for(int j=0;j<c;j++)
			{
				double val = mat[i][j];
				if(val<1e-13)
				{
					if(i==j)
					{
						new_mat[i][j] = 1e-13;
					}
					else
					{
						new_mat[i][j] = 1e-13;
					}
				}
				else
				{
					new_mat[i][j] = val;
				}
			}
		}
		return new_mat;
	}
	
	public static SimpleMatrix check(SimpleMatrix mat)
	{
		int r = mat.numRows();
		int c = mat.numCols();
		SimpleMatrix new_mat = new SimpleMatrix(r,c);
		for(int i=0;i<r;i++)
		{
			for(int j=0;j<c;j++)
			{
				double val = mat.get(i,j);
				if(val<1e-13)
				{
					if(i==j)
					{
						new_mat.set(i,j,1e-13);
					}
					else
					{
						new_mat.set(i,j,1e-13);
					}
				}
				else
				{
					new_mat.set(i,j,val);
				}
			}
		}
		return new_mat;
	}
	
	// Function to find
    // cross product of two vector array.
    public static double[] crossProduct(double vect_A[], double vect_B[])
 
    {
    	double cross_P[] = new double[3];
        cross_P[0] = vect_A[1] * vect_B[2]
                     - vect_A[2] * vect_B[1];
        cross_P[1] = vect_A[2] * vect_B[0]
                     - vect_A[0] * vect_B[2];
        cross_P[2] = vect_A[0] * vect_B[1]
                     - vect_A[1] * vect_B[0];
        return cross_P;
    }
}
