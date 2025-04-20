package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import frc.robot.utility.PositionEstimator;
import frc.robot.Constants.UltrasonicConstants.UltrasonicSide;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;


public class PositionGetterSubsystem extends SubsystemBase {
    private final UltrasonicSubsystem[] allSonics;

    private final Pigeon2 gyro;

    private final PositionEstimator position_estimator = new PositionEstimator();

    private final int MAX_READINGS_SIZE = 10;
    private final double MAX_POSITION_DEVIATION = 200; // 20cm for robot position deviation
    private final double SMOOTHING_FACTOR = 0.7;

    private final double OBJECT_DETECTION_TOLERANCE = 150;

    private final Map<String, List<double[]>> recent_positions = new HashMap<>();
    private double[] estimated_robot_position = new double[] {0, 0};
    private boolean is_first_estimate = true;
    private double estimation_confidence;

    double current_angle_degree;
    double current_angle_radian;

    public PositionGetterSubsystem(Pigeon2 gyro, UltrasonicSubsystem[] sonicArray) {
        this.gyro = gyro;

        allSonics = sonicArray;
    }

    public double getEstimationConfidence() {
        return this.estimation_confidence;
    }

    public double[] getRobotEstimatedPosition() {
        return this.estimated_robot_position;
    }

    public void resetRobotEstimation() {
        estimated_robot_position = new double[] {0, 0};
        is_first_estimate = true;
        estimation_confidence = 0.1;

        for (UltrasonicSubsystem sonic : allSonics) {
            sonic.clear();
        }

        position_estimator.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // math only takes radians
        current_angle_degree = this.gyro.getAngle() % 360;
        if (current_angle_degree<0) current_angle_degree += 360;
        current_angle_radian = Math.toRadians(current_angle_degree);

        SmartDashboard.putNumber("gyro", current_angle_degree);

        // process all sonic's estimation on the robot's position based on the object detected
        calculateSonicPosition();

        // estimate robot position based on the sonic's readings
        // estimated_robot_position();
    }

    private void calculateSonicPosition() {
        for (UltrasonicSubsystem sonic : allSonics) {
            String name = sonic.getSide().name();

            double[] xy = sonic.calculatePosition(current_angle_radian);

            if (xy.length == 1) {
                SmartDashboard.putNumber(String.join(" ", name, "sonic: X"), -1);
                SmartDashboard.putNumber(String.join(" ", name, "sonic: Y"), -1);
                continue;
            }

            SmartDashboard.putNumber(String.join(" ", name, "sonic: X"), xy[0]);
            SmartDashboard.putNumber(String.join(" ", name, "sonic: Y"), xy[1]);

            insertPosition(name, xy);
        }
    }

    private void insertPosition(String name, double[] position) {
        // invalid input
        if (position.length == 1 && position[0] == -1) {
            return;
        }

        // create new arraylist if not already existed
        recent_positions.computeIfAbsent(name, k -> new ArrayList<>());

        List<double[]> sensor_position = recent_positions.get(name);
        sensor_position.add(position);

        // remove first if size exceeded limit
        if (sensor_position.size() > MAX_READINGS_SIZE) {
            sensor_position.remove(0);
        }
    }

    private void estimated_robot_position() {
        if (this.current_angle_degree > 45 || this.current_angle_degree < 315) return;

        System.out.println(estimated_robot_position);
        System.out.println(estimation_confidence);
        System.out.println();

        double output_x = -1;
        double output_y = -1;

        List<double[]> front = recent_positions.get("FRONT");

        output_y = positionListGetXY(front, GET_XY.GET_Y);

        List<double[]> left_front  = recent_positions.get("LEFT_FRONT");
        List<double[]> left_back   = recent_positions.get("LEFT_BACK");
        List<double[]> right_front = recent_positions.get("RIGHT_FRONT");
        List<double[]> right_back  = recent_positions.get("RIGHT_BACK");

        double left_front_x  = positionListGetXY(left_front,  GET_XY.GET_X);
        double left_back_x   = positionListGetXY(left_back,   GET_XY.GET_X);
        double right_front_x = positionListGetXY(right_front, GET_XY.GET_X);
        double right_back_x  = positionListGetXY(right_back,  GET_XY.GET_X);

        // Calculate Y positions to check if object is parallel
        double left_front_y = positionListGetXY(left_front, GET_XY.GET_Y);
        double left_back_y = positionListGetXY(left_back, GET_XY.GET_Y);
        double right_front_y = positionListGetXY(right_front, GET_XY.GET_Y);
        double right_back_y = positionListGetXY(right_back, GET_XY.GET_Y);

        double left_x = -1;
        double right_x = -1;
    
        // Check left side readings
        if (Math.abs(left_front_x - left_back_x) < OBJECT_DETECTION_TOLERANCE) {
            // Additional checks for wall vs object
            double y_diff = Math.abs(left_front_y - left_back_y);
            double expected_y_diff = Math.abs(UltrasonicSide.LEFT_FRONT.getOffset()[1] - 
                                            UltrasonicSide.LEFT_BACK.getOffset()[1]);
            
            // If Y difference matches expected sensor offset (accounting for some tolerance)
            if (Math.abs(y_diff - expected_y_diff) < OBJECT_DETECTION_TOLERANCE) {
                left_x = (left_front_x + left_back_x) / 2;
            } else {
                left_x = Math.max(left_front_x, left_back_x);
            }
        }
    
        // Check right side readings
        if (Math.abs(right_front_x - right_back_x) < OBJECT_DETECTION_TOLERANCE) {
            double y_diff = Math.abs(right_front_y - right_back_y);
            double expected_y_diff = Math.abs(UltrasonicSide.RIGHT_FRONT.getOffset()[1] - 
                                            UltrasonicSide.RIGHT_BACK.getOffset()[1]);
            
            if (Math.abs(y_diff - expected_y_diff) < OBJECT_DETECTION_TOLERANCE) {
                right_x = (right_front_x + right_back_x) / 2;
            } else {
                right_x = Math.max(right_front_x, right_back_x);
            }
        }
    
        // confidence
        if (left_front_x != -1) estimation_confidence += 0.5;
        if (left_back_x != -1) estimation_confidence += 0.5;
        if (right_front_x != -1) estimation_confidence += 0.5;
        if (right_back_x != -1) estimation_confidence += 0.5;
        if (estimation_confidence > 1) estimation_confidence = 1.0;

        // Calculate final X position
        if (left_x != -1 && right_x != -1) {
            output_x = (left_x + right_x) / 2;
        } else if (left_x != -1) {
            output_x = left_x - 2000;
            estimation_confidence *= 0.9;
        } else if (right_x != -1) {
            output_x = right_x + 2000;
            estimation_confidence *= 0.9;
        }
    
        // Penalize inconsistent readings
        if (left_front_x != -1 && left_back_x != -1) {
            double left_deviation = Math.abs(left_front_x - left_back_x);
            estimation_confidence *= Math.max(0.5, 1.0 - (left_deviation / MAX_POSITION_DEVIATION));
        }
        
        if (right_front_x != -1 && right_back_x != -1) {
            double right_deviation = Math.abs(right_front_x - right_back_x);
            estimation_confidence *= Math.max(0.5, 1.0 - (right_deviation / MAX_POSITION_DEVIATION));
        }

        if (is_first_estimate) estimation_confidence *= 0.6;

        if (is_first_estimate) estimation_confidence *= 0.6;

        // Update position if we have valid readings
        if (output_x != -1 && output_y != -1) {
            if (is_first_estimate) {
                estimation_confidence *= 0.6;
                estimated_robot_position = new double[]{output_x, output_y};
                is_first_estimate = false;
            } else {
                // todo: creating better smoothing
                estimated_robot_position[0] = SMOOTHING_FACTOR * estimated_robot_position[0] + (1 - SMOOTHING_FACTOR) * output_x;
                estimated_robot_position[1] = SMOOTHING_FACTOR * estimated_robot_position[1] + (1 - SMOOTHING_FACTOR) * output_y;
            }
        }
    
        // Update dashboard
        SmartDashboard.putNumber("Estimated X", estimated_robot_position[0]);
        SmartDashboard.putNumber("Estimated Y", estimated_robot_position[1]);
        SmartDashboard.putNumber("Confidence",  estimation_confidence);
    }

    private enum GET_XY {
        GET_X,
        GET_Y
    }
    private double positionListGetXY(List<double[]> l, GET_XY get) {
        if (l==null || l.size() < 2) return -1;

        int index = (get == GET_XY.GET_Y)? 1: 0; 

        double sum = 0;
        int size = l.size();
        for (double[] xy : l) {
            sum += xy[index];
        }

        return sum / size;
    }
}
