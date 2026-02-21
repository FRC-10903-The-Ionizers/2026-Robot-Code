package frc.robot.util;

import frc.robot.Constants;

public class InterpolationMatrix {

    private static final double[] whittakerSmoothedPositions;
    private static final double[] whittakerSmoothedValues;

    static {
        whittakerSmoothedPositions = whittakerSmoothing(Constants.ShooterConstants.kPositions);
        whittakerSmoothedValues = whittakerSmoothing(Constants.ShooterConstants.kValues);
    }

    private static double[] whittakerSmoothing(double[] x) {
        int n = x.length;
        int d = 2;
        
        // Build the d-th order difference matrix D of size (n-d) x n
        // by applying finite differences d times to an identity matrix
        double[][] D = new double[n][n];
        for (int i = 0; i < n; i++)
            D[i][i] = 1.0; // start as identity

        for (int pass = 0; pass < d; pass++) {
            int rows = n - pass - 1;
            double[][] diff = new double[rows][n];
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < n; j++) {
                    diff[i][j] = D[i + 1][j] - D[i][j];
                }
            }
            D = diff;
        }

        // Compute A = I + lambda * D'D
        int dRows = D.length;
        double[][] A = new double[n][n];
        for (int i = 0; i < n; i++)
            A[i][i] = 1.0; // identity

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double dtd = 0.0;
                for (int k = 0; k < dRows; k++) {
                    dtd += D[k][i] * D[k][j]; // D^T * D
                }
                A[i][j] += Constants.ShooterConstants.kInterpolationLambda * dtd;
            }
        }

        // Solve A*z = y via Gaussian elimination with partial pivoting
        // Augment A with y
        double[][] aug = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++)
                aug[i][j] = A[i][j];
            aug[i][n] = x[i];
        }

        for (int col = 0; col < n; col++) {
            // Partial pivot
            int maxRow = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(aug[row][col]) > Math.abs(aug[maxRow][col]))
                    maxRow = row;
            }
            double[] tmp = aug[col];
            aug[col] = aug[maxRow];
            aug[maxRow] = tmp;

            // Eliminate below
            for (int row = col + 1; row < n; row++) {
                double factor = aug[row][col] / aug[col][col];
                for (int j = col; j <= n; j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        // Back substitution
        double[] z = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            z[i] = aug[i][n];
            for (int j = i + 1; j < n; j++) {
                z[i] -= aug[i][j] * z[j];
            }
            z[i] /= aug[i][i];
        }

        return z;
    }

    public static double interpolate(double point) {

        if (point <= whittakerSmoothedPositions[0]){
            double[] p1 = {whittakerSmoothedPositions[0], whittakerSmoothedValues[0]};
            double[] p2 = {whittakerSmoothedPositions[1], whittakerSmoothedValues[1]};
            double slope = (p2[1] - p1[1])/(p2[0] - p1[0]);
            return slope*point;
        }

        if (point >= whittakerSmoothedPositions[whittakerSmoothedPositions.length - 1]){
            double[] p1 = {whittakerSmoothedPositions[whittakerSmoothedPositions.length - 2], whittakerSmoothedValues[whittakerSmoothedValues.length - 2]};
            double[] p2 = {whittakerSmoothedPositions[whittakerSmoothedPositions.length - 1], whittakerSmoothedValues[whittakerSmoothedValues.length - 1]};
            double slope = (p2[1] - p1[1])/(p2[0] - p1[0]);
            return slope*point;
        }

        int left = 0;
        int right = whittakerSmoothedPositions.length - 1;

        while (left < right - 1) {
            int mid = (left + right) / 2;
            if (whittakerSmoothedPositions[mid] <= point) {
                left = mid;
            } else {
                right = mid;
            }
        }

        double fraction = (point - whittakerSmoothedPositions[left]) / (whittakerSmoothedPositions[right] - whittakerSmoothedPositions[left]);
        return whittakerSmoothedValues[left] * (1 - fraction) + whittakerSmoothedValues[right] * fraction;
    }
}
