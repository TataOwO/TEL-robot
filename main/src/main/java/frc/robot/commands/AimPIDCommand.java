// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MecanumDriveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;

/** An example command that uses an example subsystem. */
public class AimPIDCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MecanumDriveSubsystem m_drive;
  private final Pigeon2 gyro;

  private double target_degree;

  // modified by `execute()`, viewed by `isFinished()`
  private WheelSpeeds current_wheel_speeds;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_drive The subsystem used by this command.
   */
  public AimPIDCommand(MecanumDriveSubsystem m_drive, Pigeon2 gyro, double target_degree) {
    this.m_drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);

    this.gyro = gyro;
    // this.gyro.reset();

    this.target_degree = gyro.getAngle() + target_degree;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degree = target_degree-gyro.getAngle();
    double degree10 = degree*0.0027777777;

    System.out.println(String.format("%f %f", degree, degree10));

    current_wheel_speeds = MecanumDrive.driveCartesianIK(0, 0, degree10);

    m_drive.drivePositionClosedLoop(degree10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(target_degree-gyro.getAngle()) > 5) return false;
    if (current_wheel_speeds.frontLeft  > 0.1) return false;
    if (current_wheel_speeds.frontRight > 0.1) return false;
    if (current_wheel_speeds.rearLeft   > 0.1) return false;
    if (current_wheel_speeds.rearRight  > 0.1) return false;

    return true;
  }
}
