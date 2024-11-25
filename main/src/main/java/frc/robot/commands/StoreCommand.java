package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.StorageSubsystem;

public class StoreCommand extends Command {
    // private final StorageSubsystem m_storage;

    private final StorageSubsystem[] m_storages;
    private final double RPM;

    private final Timer timer = new Timer();

    private final boolean is_store;

    private int button_pressed_count;

    public StoreCommand(StorageSubsystem[] storages, boolean is_store) {
        m_storages = storages;

        this.is_store = is_store;

        if (is_store) RPM = StorageConstants.STORAGE_RPM;
        else RPM = StorageConstants.REVERSE_RPM;
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        button_pressed_count = 0;
    }

    @Override
    public void execute() {
        for (StorageSubsystem storage : m_storages) {
            if (storage.buttonPressed() && timer.get()>0.3) storage.stop();
            else storage.setStore(RPM);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) return;
        
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

        if (button_pressed) ++button_pressed_count;

        return is_store  && button_pressed_count>50 || 
               !is_store && timer.get() > 0.7
               ;
    }
}
