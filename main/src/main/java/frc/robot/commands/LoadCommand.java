package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.TransporterConstants;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TransportDirectionSubsystem;
import frc.robot.subsystems.TransporterSubsystem;

public class LoadCommand extends Command {
    // private final StorageSubsystem m_storage;

    private final LoaderSubsystem m_left_loader;
    private final LoaderSubsystem m_right_loader;
    private final TransporterSubsystem m_transport;

    private LoaderSubsystem m_loader;

    private boolean is_left = false;

    private final Timer timer = new Timer();

    public LoadCommand(LoaderSubsystem left_loader, LoaderSubsystem right_loader, TransporterSubsystem transport) {
        this.m_left_loader = left_loader;
        this.m_right_loader = right_loader;
        this.m_transport = transport;
    }
    
    public boolean isLeftLoader() {
        return this.is_left;
    }

    @Override
    public void initialize() {
        final int right_disc_count = this.m_right_loader.getDiscCount();
        final int left_disc_count  = this.m_left_loader .getDiscCount();

        // if (left_disc_count < right_disc_count) {
        //     // m_storage = right_storage;
        //     m_loader  = this.m_right_loader;
        //     is_left = false;
        // } else {
        //     // m_storage = left_storage;
        //     m_loader  = this.m_left_loader;
        //     is_left = true;
        // }

        m_loader  = this.m_right_loader;
        is_left = true;

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_loader.load(-LoaderConstants.LOADER_SPEED);
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
        return this.timer.get() > 1.0;
    }
}
