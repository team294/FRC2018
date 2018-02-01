package org.usfirst.frc.team294.utilities;

import java.util.Arrays;

public class VelocityChecker {
	private double[] histArray;
	private int histArrayIndex = 0;

	public VelocityChecker(double secs) {
		histArray = new double[(int)(secs*50)];
		Arrays.fill(histArray, Double.MAX_VALUE); // Dont forget, histArray is filled with big numbers to skew avergae
													// at start
	}

	private VelocityChecker() {}

	public static VelocityChecker getBySize(int size) {
		VelocityChecker temp = new VelocityChecker();
		temp.histArray = new double[size];
		return temp;
	}

	public void addValue(double val) {
		histArray[histArrayIndex] = val;
		histArrayIndex = ++histArrayIndex % histArray.length;
	}

	public void clearHistory() {
		Arrays.fill(histArray, Double.MAX_VALUE); // Dont forget, histArray is filled with big numbers to skew avergae
													// at start
	}

	public double getAverage() {
		double sum = 0;
		for (double d : histArray) {
			sum += d;
		}
		return sum / histArray.length;
	}
}
