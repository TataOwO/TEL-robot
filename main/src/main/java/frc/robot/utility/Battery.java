package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Battery extends SubsystemBase {
    List<Double> history = new ArrayList<>();
    private final int LENGTH = 20;

    public Battery() {}

    public double get() {
        double sum = 0;
        for (double v : history) {
            sum += v;
        }

        SmartDashboard.putNumber("battery", sum/history.size());

        return sum / history.size();
    }

    @Override
    public void periodic() {
        history.add(PowerJNI.getVinVoltage());
        if (history.size() > LENGTH) history.remove(0);
    }
}
