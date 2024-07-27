// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MecanumDriveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/** An example command that uses an example subsystem. */
public class AimCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MecanumDriveSubsystem m_drive;
  private final Pigeon2 gyro;

  private double targetDegree;

  // modified by `execute()`, viewed by `isFinished()`
  private WheelSpeeds currentWheelSpeeds;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_drive The subsystem used by this command.
   */
  public AimCommand(MecanumDriveSubsystem m_drive, Pigeon2 gyro, double targetDegree) {
    this.m_drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);

    this.gyro = gyro;
    this.gyro.reset();

    this.targetDegree = targetDegree;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degree = targetDegree-gyro.getAngle();
    double degree10 = degree*0.0027777777;

    System.out.println(String.format("%f %f", degree, degree10));

    currentWheelSpeeds = MecanumDrive.driveCartesianIK(0, 0, degree10);

    m_drive.drivePositionClosedLoop(degree10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(targetDegree-gyro.getAngle()) > 5) return false;
    if (currentWheelSpeeds.frontLeft  > 0.1) return false;
    if (currentWheelSpeeds.frontRight > 0.1) return false;
    if (currentWheelSpeeds.rearLeft   > 0.1) return false;
    if (currentWheelSpeeds.rearRight  > 0.1) return false;

    return true;
  }
}
