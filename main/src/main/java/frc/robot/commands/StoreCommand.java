package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.StorageSubsystem;

public class StoreCommand extends Command {
    private final StorageSubsystem m_storage;
    private final double RPM;

    private final Timer timer = new Timer();

    private final boolean is_store;

    private int button_pressed_count;

    public StoreCommand(StorageSubsystem storage, boolean is_store) {
        m_storage = storage;

        this.is_store = is_store;

        if (is_store) RPM = StorageConstants.STORAGE_RPM;
        else RPM = StorageConstants.REVERSE_RPM;

        addRequirements(storage);
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        button_pressed_count = 0;
    }

    @Override
    public void execute() {
        if (m_storage.buttonPressed() && timer.get()>0.3) m_storage.stop();
        else m_storage.setStore(RPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) return;
        
        m_storage.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean button_pressed = m_storage.buttonPressed();

        if (button_pressed) ++button_pressed_count;

        return is_store  && button_pressed_count>50 || 
               !is_store && timer.get() > 0.7
               ;
    }
}
