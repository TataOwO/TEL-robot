package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StorageSubsystem;

public class LoadCommand extends Command {
    private final StorageSubsystem m_left_storage;
    private final StorageSubsystem m_right_storage;

    LoadCommand(StorageSubsystem left_storage, StorageSubsystem right_storage) {
        this.m_left_storage  = left_storage;
        this.m_right_storage = right_storage;
    }
}
