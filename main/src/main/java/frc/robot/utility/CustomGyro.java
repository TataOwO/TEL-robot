package frc.robot.utility;

import com.ctre.phoenix6.hardware.Pigeon2;

public class CustomGyro extends Pigeon2 {
    public CustomGyro(int id) {
        super(id);
    }

    @Override
    public double getAngle() {
        return super.getAngle();
    }
}
