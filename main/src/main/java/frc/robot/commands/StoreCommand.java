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

    private final StorageSubsystem[] m_storages;

    private final Timer timer = new Timer();

    public StoreCommand(StorageSubsystem[] storages) {
        m_storages = storages;
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        for (StorageSubsystem storage : m_storages) {
            if (storage.buttonPressed()) storage.stop();
            else storage.setStore(Constants.StorageConstants.STORAGE_RPM);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        for (StorageSubsystem storage : m_storages) {
            if (storage.buttonPressed()) storage.stop();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
