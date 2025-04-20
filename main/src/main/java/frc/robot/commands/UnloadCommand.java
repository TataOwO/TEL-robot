package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.LoaderSubsystem;

public class UnloadCommand extends Command {
    private final LoaderSubsystem m_loader;

    private final Timer timer = new Timer();

    public UnloadCommand(LoaderSubsystem loader) {
        this.m_loader = loader;

        addRequirements(m_loader);
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    public double time() {
        return timer.get();
    }

    @Override
    public void execute() {
        m_loader.reverse();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_loader.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return time() > LoaderConstants.LOAD_MINIMUM_TIME;
    }
}
