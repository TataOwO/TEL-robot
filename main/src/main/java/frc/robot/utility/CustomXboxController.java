package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CustomXboxController extends CommandXboxController {
    public CustomXboxController(int port) {
        super(port);
    }
    
    @Override
    public double getLeftX() {
        double val = super.getLeftX();
        if (val > -0.1 && val < 0.1) val = 0;
        return val;
    }

    @Override
    public double getLeftY() {
        double val = super.getLeftY();
        if (val > -0.1 && val < 0.1) val = 0;
        return val;
    }

    @Override
    public double getRightX() {
        double val = super.getRightX();
        if (val > -0.1 && val < 0.1) val = 0;
        return val;
    }

    @Override
    public double getRightY() {
        double val = super.getRightY();
        if (val > -0.1 && val < 0.1) val = 0;
        return val;
    }
}
