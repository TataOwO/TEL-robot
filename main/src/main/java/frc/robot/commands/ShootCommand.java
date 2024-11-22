// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TransporterSubsystem;
import frc.robot.Constants.TransporterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem     shooter_subsystem;
  private final TransporterSubsystem transport_subsystem;
  private final Timer timer = new Timer();
  private double current_time = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter_subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooter_subsystem, TransporterSubsystem transport_subsystem) {
    this.shooter_subsystem       = shooter_subsystem;
    this.transport_subsystem  = transport_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter_subsystem, transport_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_time = timer.get();
    double shooter_speed = SmartDashboard.getNumber("shooter speed", 0);
    double support_speed = Math.min(shooter_speed*0.02+0.2, 1.0);

    // shooter 120 RPS (this will depend on the distance)
    // transport 0.4/1
    if (current_time < 1) {
      shooter_subsystem.shoot(
        shooter_speed,
        support_speed
      );
      transport_subsystem.stop();
    } else if (current_time < 2.5) {
      shooter_subsystem.shoot(
        shooter_speed,
        support_speed
      );
      transport_subsystem.transport();
    } else if (current_time >= 2.5) {
      shooter_subsystem.stop();
      transport_subsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_subsystem.stop();
    transport_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (current_time >= 2.5) return true;
    return false;
  }
}
