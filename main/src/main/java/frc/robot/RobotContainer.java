// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Constants.UltrasonicConstants.ultrasonicSide;
import frc.robot.commands.AimPIDCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.XboxController;

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
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final LoaderSubsystem m_frontLoaderSubsystem = new LoaderSubsystem(LoaderConstants.loaderSide.FRONT);
  private final LoaderSubsystem m_leftLoaderSubsystem = new LoaderSubsystem(LoaderConstants.loaderSide.LEFT);
  private final TransporterSubsystem m_transporterSubsystem = new TransporterSubsystem();
  private final UltrasonicSubsystem ultrasonic_subsystem = new UltrasonicSubsystem(UltrasonicConstants.ultrasonicSide.TEST);

  private XboxController controller;// = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private final Pigeon2 gyro = new Pigeon2(16);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private CommandXboxController m_driverController;// =
      //new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    compressor.enableDigital();
    compressor.disable();

    System.out.println(gyro.getAngle());
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
    // shooter
    // m_driverController.y().onTrue(m_shooterSubsystem.shootCommand(120));
    // m_driverController.y().onFalse(m_shooterSubsystem.stopCommand());

    // // left loader
    // m_driverController.x().onTrue(m_leftLoaderSubsystem.loadCommand(0.4));
    // m_driverController.x().onFalse(m_leftLoaderSubsystem.stopCommand());

    // // front loader
    // m_driverController.b().onTrue(m_frontLoaderSubsystem.loadCommand(0.4));
    // m_driverController.b().onFalse(m_frontLoaderSubsystem.stopCommand());

    // // transporter
    // m_driverController.a().onTrue(m_transporterSubsystem.transportCommand(0.4));
    // m_driverController.a().onFalse(m_transporterSubsystem.stopCommand());

    AimCommand turnLeft  = new AimCommand(m_driveSubsystem, gyro, -90);
    AimCommand turnRight = new AimCommand(m_driveSubsystem, gyro, +90);

    // drive movement (WIP)
    // m_driverController.back(). toggleOnTrue(turnLeft) ;
    // m_driverController.start().toggleOnTrue(turnRight);

    // drive movement
    /* new Trigger(() -> {
      return Math.abs(m_driverController.getLeftY())>0.1 &&
        Math.abs(m_driverController.getLeftX())>0.1 &&
        Math.abs(m_driverController.getRightX())>0.1 &&
        !turnRight.isScheduled() &&
        !turnLeft. isScheduled();
    }).onTrue */
    // m_driveSubsystem.setDefaultCommand(m_driveSubsystem.run(() -> m_driveSubsystem.drive(
    //   m_driverController.getLeftY(),
    //   m_driverController.getLeftX(),
    //   -m_driverController.getRightX()
    // )));

    // drive change speed modifier
    // m_driverController.leftBumper()
    //   .onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.decreaseSpeed()));
    // m_driverController.rightBumper()
    //   .onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.increaseSpeed()));
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
