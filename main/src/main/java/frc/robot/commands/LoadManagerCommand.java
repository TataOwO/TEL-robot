package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LoaderConstants;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TransportDirectionSubsystem;
import frc.robot.subsystems.TransporterSubsystem;

public class LoadManagerCommand extends Command {
    private final TransportDirectionSubsystem transport_dir;
    private final TransporterSubsystem transport;

    private final LoaderSubsystem loader_left;
    private final LoaderSubsystem loader_right;

    private final StorageSubsystem storage_left;
    private final StorageSubsystem storage_right;

    private final Command m_left_command;
    private final Command m_right_command;

    public LoadManagerCommand(TransporterSubsystem transport, TransportDirectionSubsystem transport_dir, LoaderSubsystem load_left, LoaderSubsystem load_right, StorageSubsystem storage_left, StorageSubsystem storage_right) {
      this.transport_dir = transport_dir;
      this.transport = transport;

      loader_left  = load_left;
      loader_right = load_right;

      this.storage_left  = storage_left;
      this.storage_right = storage_right;

      addRequirements(transport_dir, load_left, load_right, storage_left, storage_right);

      TransportDirectionCommand transport_dir_shoot_command = new TransportDirectionCommand(transport_dir, true);
      TransportDirectionCommand transport_dir_load_command = new TransportDirectionCommand(transport_dir, false);

      m_left_command = Commands.sequence(
        transport_dir_load_command,
        Commands.parallel(
          new LoadCommand(load_left, transport),
          new StoreCommand(storage_left, true)
        ),
        new TransportLoadForSomeTime(transport),
        Commands.parallel(
          new LoadCommand(load_left, transport),
          new StoreCommand(storage_left, true)
        ),
        Commands.parallel(
          new StoreCommand(storage_left, false),
          transport_dir_shoot_command
        )
      );

      m_right_command = Commands.sequence(
        transport_dir_load_command,
        Commands.parallel(
          new LoadCommand(load_right, transport),
          new StoreCommand(storage_right, true)
        ),
        new TransportLoadForSomeTime(transport),
        Commands.parallel(
          new LoadCommand(load_right, transport),
          new StoreCommand(storage_right, true)
        ),
        Commands.parallel(
          new StoreCommand(storage_right, false),
          transport_dir_shoot_command
        )
      );
    }

    @Override
    public void initialize() {
      if (loader_right.getDiscCount() > 0) {
        m_right_command.schedule();
      }
      else {
        m_left_command.schedule();
      }
    }

    @Override
    public void end(boolean interrupted) {
      m_right_command.end(interrupted);
      m_left_command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_right_command.isFinished() && m_left_command.isFinished();
    }

    private class TransportLoadForSomeTime extends Command {
      private final Timer timer = new Timer();
      private final TransporterSubsystem transport;

      public TransportLoadForSomeTime(TransporterSubsystem transport) {
        this.transport = transport;
      }

      @Override
      public void initialize() {
        timer.reset();
        timer.start();
        transport.load();
      }

      @Override
      public void end(boolean interrupted) {
        transport.stop();
      }

      @Override
      public boolean isFinished() {
          return timer.get() > LoaderConstants.TRANSPORT_LOAD_TIME;
      }
    }
}
