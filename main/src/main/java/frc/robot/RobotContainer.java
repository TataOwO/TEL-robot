// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.MotorSafety;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final MecanumDriveSubsystem m_driveSubsystem = new MecanumDriveSubsystem();
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();


    // MotorSafety ms = new MotorSafety();
    // ms.setSafetyEnabled(false);

    m_driveSubsystem.setDefaultCommand(Commands.run(() -> m_driveSubsystem.driveWithSpeed(
      m_driverController.getLeftY(),
      m_driverController.getLeftX(),
      m_driverController.getRightX(),
      m_driveSubsystem.getSpeedModifier()
    ), m_driveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // new Trigger(() -> Math.abs(m_driverController.getLeftX())>0.1 && Math.abs(m_driverController.getLeftY())>0.1 && Math.abs(m_driverController.getRightY())>0.1)
    //     .whileTrue(Commands.run(() -> m_driveSubsystem.driveWithSpeed(
    //       m_driverController.getLeftY(),
    //       -m_driverController.getLeftX(),
    //       m_driverController.getRightY(),
    //       m_driveSubsystem.getSpeedModifier()
    //     )));

    m_driverController.leftBumper()
      .onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.decreaseSpeed()));
    m_driverController.rightBumper()
      .onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.increaseSpeed()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
