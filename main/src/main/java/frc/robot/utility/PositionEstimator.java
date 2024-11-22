package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;

public class PositionEstimator {
    // Constants for position prediction
    private static final double MAX_VELOCITY = 500.0; // mm per second
    private static final double MAX_ACCELERATION = 250.0; // mm per second^2
    private static final double DT = 0.02; // 20ms robot period
    
    // State tracking
    private double[] lastPosition = new double[]{0, 0};
    private double[] velocity = new double[]{0, 0};
    private double[] lastVelocity = new double[]{0, 0};
    private long lastTimestamp = System.currentTimeMillis();

    // Confidence-based smoothing parameters
    private static final double MIN_SMOOTHING = 0.3;
    private static final double MAX_SMOOTHING = 0.9;
    private static final double PREDICTION_WEIGHT = 0.3;

    public double calculateDynamicSmoothingFactor(double confidence, double[] measuredPosition, double[] predictedPosition) {
        // Base smoothing factor scaled by confidence
        double baseSmoothingFactor = MIN_SMOOTHING + (MAX_SMOOTHING - MIN_SMOOTHING) * confidence;
        
        // Calculate prediction error
        double predictionError = calculatePredictionError(measuredPosition, predictedPosition);
        
        // Adjust smoothing based on prediction error (lower error = higher smoothing)
        double predictionAdjustment = Math.max(0, 1 - predictionError / 1000) * PREDICTION_WEIGHT;
        
        // Combine base smoothing with prediction adjustment
        return Math.min(MAX_SMOOTHING, baseSmoothingFactor + predictionAdjustment);
    }

    public double[] predictNextPosition(double[] currentPosition) {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTimestamp) / 1000.0;
        
        // Update velocity
        for (int i = 0; i < 2; i++) {
            double newVelocity = (currentPosition[i] - lastPosition[i]) / deltaTime;
            // Apply velocity limits
            newVelocity = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, newVelocity));
            
            // Calculate acceleration
            double acceleration = (newVelocity - lastVelocity[i]) / deltaTime;
            // Apply acceleration limits
            acceleration = Math.max(-MAX_ACCELERATION, Math.min(MAX_ACCELERATION, acceleration));
            
            // Update velocity with limited acceleration
            velocity[i] = lastVelocity[i] + acceleration * deltaTime;
        }
        
        // Predict next position
        double[] predictedPosition = new double[2];
        for (int i = 0; i < 2; i++) {
            predictedPosition[i] = currentPosition[i] + velocity[i] * DT;
        }
        
        // Update state
        lastPosition = Arrays.copyOf(currentPosition, 2);
        lastVelocity = Arrays.copyOf(velocity, 2);
        lastTimestamp = currentTime;
        
        return predictedPosition;
    }

    private double calculatePredictionError(double[] measured, double[] predicted) {
        if (measured == null || predicted == null) return Double.MAX_VALUE;
        double dx = measured[0] - predicted[0];
        double dy = measured[1] - predicted[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    public void reset() {
        lastPosition = new double[]{0, 0};
        velocity = new double[]{0, 0};
        lastVelocity = new double[]{0, 0};
        lastTimestamp = System.currentTimeMillis();
    }
}