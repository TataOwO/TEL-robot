package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.opencv.core.Mat;

import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.Constants.UltrasonicConstants.UltrasonicSide;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;


public class PositionGetterSubsystem extends SubsystemBase {
    private final ArrayList<UltrasonicSubsystem> allSonics = new ArrayList<>();
    private final ArrayList<String> sonicNameList = new ArrayList<>();

    private final Pigeon2 gyro;

    private final int MAX_READINGS_SIZE = 10;
    private final double MAX_POSITION_DEVIATION = 200; // 20cm for robot position deviation
    private final double SMOOTHING_FACTOR = 0.7;

    private final double WALL_TOLERANCE = 150;

    private final Map<String, List<double[]>> recent_positions = new HashMap<>();
    private double[] estimated_robot_position = new double[] {0, 0};
    private boolean is_first_estimate = true;

    double current_angle_degree;
    double current_angle_radian;

    public PositionGetterSubsystem(Pigeon2 gyro, UltrasonicSubsystem[] sonicArray) {
        this.gyro = new Pigeon2(gyro.getDeviceID());

        for (UltrasonicSubsystem sonic : sonicArray) {
            UltrasonicSide side = sonic.getSide();
            allSonics.add(sonic);
            sonicNameList.add(sonic.getName());
        }
    }

    private static class Wallhacks {
        private static final double LEFT_WALL_X = 0;
        private static final double RIGHT_WALL_X = 4000;
        private static final double FRONT_WALL_Y = 0;
        private static final double WALL_TOLERANCE = 150; // 15cm

        double[] position;      // Detected position
        double distance;        // Raw distance reading
        double detectionAngle; // Absolute angle of detection
        WallType wallType;     // Which wall was detected
        double confidence;     // Confidence in this reading

        Wallhacks(double[] position, double distance, double detectionAngle, WallType wallType, double confidence) {
            this.position = position;
            this.distance = distance;
            this.detectionAngle = detectionAngle;
            this.wallType = wallType;
            this.confidence = confidence;
        }
    }

    private enum GET_XY {
        GET_X,
        GET_Y
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // math only takes radians
        current_angle_degree = this.gyro.getAngle() % 360;
        current_angle_radian = Math.toRadians(current_angle_degree);

        SmartDashboard.putNumber("gyro", current_angle_radian);

        // process all sonic's estimation on the robot's position based on the object detected
        calculateSonicPosition();

        // estimate robot position based on
        estimated_robot_position();
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

            classifyWallhacks(sonic, xy);

            SmartDashboard.putNumber(String.join(" ", name, "sonic: X"), xy[0]);
            SmartDashboard.putNumber(String.join(" ", name, "sonic: Y"), xy[1]);

            insertPosition(name, xy);
        }
    }

    private void classifyWallhacks(UltrasonicSubsystem sonic, double[] xy) {
        UltrasonicSide side = sonic.getSide(); 
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

        double output_x = -1;
        double output_y = -1;

        List<double[]> front = recent_positions.get("FRONT");

        output_y = listXy2Xy(front, GET_XY.GET_Y);

        List<double[]> left_front  = recent_positions.get("LEFT_FRONT");
        List<double[]> left_back   = recent_positions.get("LEFT_BACK");
        List<double[]> right_front = recent_positions.get("RIGHT_FRONT");
        List<double[]> right_back  = recent_positions.get("RIGHT_BACK");

        double left_front_x  = listXy2Xy(left_front,  GET_XY.GET_X);
        double left_back_x   = listXy2Xy(left_back,   GET_XY.GET_X);
        double right_front_x = listXy2Xy(right_front, GET_XY.GET_X);
        double right_back_x  = listXy2Xy(right_back,  GET_XY.GET_X);

        if (Math.abs(left_front_x-left_back_x) < WALL_TOLERANCE) {

        }
        if (Math.abs(right_front_x-right_back_x) < WALL_TOLERANCE) {

        }
    }

    private double listXy2Xy(List<double[]> l, GET_XY get) {
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
