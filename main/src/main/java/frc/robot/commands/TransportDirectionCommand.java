package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransportDirectionSubsystem;

public class TransportDirectionCommand extends Command {
    private final TransportDirectionSubsystem m_transport_dir;
    private final boolean isShootAngle;

    private final Timer timer = new Timer();

    private int button_pressed_times = 0;

    public TransportDirectionCommand(TransportDirectionSubsystem dir_subsystem, boolean isShootAngle) {
        this.m_transport_dir = dir_subsystem;
        this.isShootAngle = isShootAngle;

        addRequirements(dir_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (this.isShootAngle) this.m_transport_dir.setShoot();
        else this.m_transport_dir.setLoad();

        this.timer.reset();
        this.timer.start();

        button_pressed_times = 0;
    }

    @Override
    public void execute() {
        this.m_transport_dir.setRunDirection();

        if (m_transport_dir.getButton()) ++button_pressed_times;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.m_transport_dir.stopDirection();

        this.m_transport_dir.setReady(this.isShootAngle);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return
            // if we're trying to shoot and it's ready to shoot
            this.isShootAngle  && this.timer.get() > 1.3 ||
            // or if we're trying to load and it's ready to load
            !this.isShootAngle && button_pressed_times > 3
           ;
    }
}
