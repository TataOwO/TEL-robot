package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.TransporterSubsystem;

public class LoadCommand extends Command {
    private final TransporterSubsystem m_transport;

    private LoaderSubsystem m_loader;

    private boolean is_left = false;

    private final Timer timer = new Timer();

    public LoadCommand(LoaderSubsystem loader, TransporterSubsystem transport) {
        this.m_loader = loader;
        this.m_transport = transport;

        addRequirements(m_loader, m_transport);
    }
    
    public boolean isLeftLoader() {
        return this.is_left;
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
        m_loader.load();
        m_transport.load();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_loader.stop();
        m_loader.useDisc();
        m_transport.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_transport.getButton();
    }
}
