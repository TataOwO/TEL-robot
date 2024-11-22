package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TransportDirectionSubsystem;

public class TransportDirectionCommand extends Command {
    private final TransportDirectionSubsystem m_transport_dir;
    private final boolean isShootAngle;

    private final Timer timer = new Timer();

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

        System.out.println("Transport dir starts");
    }

    @Override
    public void execute() {
        this.m_transport_dir.setRunDirection();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Transport dir ends");

        this.m_transport_dir.stopDirection();

        this.m_transport_dir.setReady(this.isShootAngle);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return
            // if we're trying to shoot and it's ready to shoot
            // this.isShootAngle  && m_transport_dir.isReadyShoot() ||
            // or if we're trying to load and it's ready to load
            // !this.isShootAngle && m_transport_dir.isReadyLoad() ||
            this.timer.get() > 1.1;
    }
}
