// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class ExtraShootCommand extends Command {
  private final ShooterSubsystem     shooter_subsystem;
  private final Timer timer = new Timer();
  private double current_time = 0;

  private double shooter_speed = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter_subsystem The subsystem used by this command.
   */
  public ExtraShootCommand(ShooterSubsystem shooter_subsystem) {
    this.shooter_subsystem       = shooter_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    shooter_speed = 69;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_time = timer.get();
    double support_speed = 1;

    // shooter 120 RPS (this will depend on the distance)
    // transport 0.4/1
    if (current_time < 1) {
      shooter_subsystem.shoot(
        shooter_speed,
        0
      );
    } else if (current_time < 2.5) {
      shooter_subsystem.shoot(
        shooter_speed,
        support_speed
      );
    } else if (current_time >= 2.5) {
      shooter_subsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (current_time >= 2.5) return true;
    return false;
  }
}