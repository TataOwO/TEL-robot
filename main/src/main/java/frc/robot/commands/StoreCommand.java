package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TransportDirectionSubsystem;

public class StoreCommand extends Command {
    // private final StorageSubsystem m_storage;

    private final StorageSubsystem[] m_storages;
    private final double RPM;

    private final Timer timer = new Timer();

    public StoreCommand(StorageSubsystem[] storages, boolean is_store) {
        m_storages = storages;

        if (is_store) RPM = StorageConstants.STORAGE_RPM;
        else RPM = StorageConstants.REVERSE_RPM;
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
            else storage.setStore(RPM);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        for (StorageSubsystem storage : m_storages) {
            storage.stop();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean button_pressed = false;
        for (StorageSubsystem storage : m_storages) {
            button_pressed |= storage.buttonPressed();
        }

        return timer.get() > 0.3 || button_pressed;
    }
}
