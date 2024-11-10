package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import org.opencv.core.Mat;

import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.Constants.UltrasonicConstants.UltrasonicSide;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;


public class PositionGetterSubsystem extends SubsystemBase {
    private final ArrayList<UltrasonicSubsystem> allSonics = new ArrayList<>();
    private final ArrayList<String> sonicNameList = new ArrayList<>();

    // this hashmap is responsible for storing the most recent calculated position
    // the key is the sonic side's enum name
    // the arraylist stores the most recent `recentPositionCount` positions, both x and y
    private final HashMap<String, ArrayList<double[]>> positionListMap = new HashMap<>();
    private final int recentPositionCount = 10;

    private final Pigeon2 gyro;

    double current_angle_r;

    public PositionGetterSubsystem(Pigeon2 gyro, UltrasonicSubsystem[] sonicArray) {
        this.gyro = new Pigeon2(gyro.getDeviceID());

        for (UltrasonicSubsystem sonic : sonicArray) {
            UltrasonicSide side = sonic.getSide();
            allSonics.add(sonic);
            sonicNameList.add(sonic.getName());
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // we face forward by default, but x-axis faces right
        // so we're adding 90 here to solve this problem
        current_angle_r = Math.toRadians(this.gyro.getAngle() + 90);

        SmartDashboard.putNumber("gyro", current_angle_r);

        calculateSonicPosition();
    }

    private void calculateSonicPosition() {
        for (UltrasonicSubsystem sonic : allSonics) {
            String name = sonic.getSide().name();

            double[] xy = sonic.calculatePosition(current_angle_r);

            SmartDashboard.putNumber(String.join(" ", name, "sonic: X"), xy[0]);
            SmartDashboard.putNumber(String.join(" ", name, "sonic: Y"), xy[1]);

            insertPosition(name, xy);
        }
    }

    private void insertPosition(String name, double[] position) {
        
    }
}
