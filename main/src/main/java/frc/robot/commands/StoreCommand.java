package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TransportDirectionSubsystem;

public class StoreCommand extends Command {
    // private final StorageSubsystem m_storage;

    private final StorageSubsystem m_left_storage;
    private final StorageSubsystem m_right_storage;

    private LoaderSubsystem m_storage;

    private final Timer timer = new Timer();

    public StoreCommand(StorageSubsystem left_loader, StorageSubsystem right_loader) {
        this.m_left_storage = left_loader;
        this.m_right_storage = right_loader;
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (m_left_storage.buttonPressed()) m_left_storage.stop();
        else m_left_storage.setStore(Constants.StorageConstants.STORAGE_RPM);

        if (m_right_storage.buttonPressed()) m_right_storage.stop();
        else m_right_storage.setStore(Constants.StorageConstants.STORAGE_RPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_left_storage.stop();
        m_right_storage.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
