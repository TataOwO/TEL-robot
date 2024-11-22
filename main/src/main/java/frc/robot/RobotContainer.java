// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Constants.StorageConstants.StorageSide;
import frc.robot.Constants.UltrasonicConstants.UltrasonicSide;
import frc.robot.commands.AimPIDCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LoadCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StoreCommand;
import frc.robot.commands.TransportDirectionCommand;
import frc.robot.subsystems.*;
import frc.robot.utility.CustomXboxController;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // GRYO
  private final Pigeon2 gyro = new Pigeon2(OperatorConstants.GYRO_PORT);

  // DRIVE
  // private final MecanumDriveSubsystem m_drive_subsystem = new MecanumDriveSubsystem();

  // SHOOTER
  private final ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();

  // LOADERS
  private final LoaderSubsystem m_right_loader_subsystem = new LoaderSubsystem(LoaderConstants.loaderSide.RIGHT);
  private final LoaderSubsystem m_left_loader_subsystem = new LoaderSubsystem(LoaderConstants.loaderSide.LEFT);

  // TRANSPORT
  private final TransporterSubsystem m_transporter_subsystem = new TransporterSubsystem();
  private final TransportDirectionSubsystem m_transport_dir_subsystem = new TransportDirectionSubsystem();

  // STORAGE
  private final StorageSubsystem m_left_storage_subsystem  = new StorageSubsystem(StorageSide.LEFT);
  private final StorageSubsystem m_right_storage_subsystem = new StorageSubsystem(StorageSide.RIGHT);

  // ULTRASONICS
  private final PositionGetterSubsystem m_position_getter_subsystem = new PositionGetterSubsystem(gyro, new UltrasonicSubsystem[] {
    new UltrasonicSubsystem(UltrasonicConstants.UltrasonicSide.FRONT),
    new UltrasonicSubsystem(UltrasonicConstants.UltrasonicSide.LEFT_FRONT),
    new UltrasonicSubsystem(UltrasonicConstants.UltrasonicSide.RIGHT_FRONT),
  });

  // CONTROLLER
  private CustomXboxController m_driverController =
      new CustomXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    gyro.reset();

    SmartDashboard.putNumber("shooter speed", 100);
    SmartDashboard.putNumber("shooter support speed", 0.4);
    SmartDashboard.putNumber("transport dir", TransporterConstants.DIRECTION_ROTATION_SPEED);
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
    // y => shooter
    m_driverController.y().toggleOnTrue(new ShootCommand(m_shooter_subsystem, m_transporter_subsystem));

    // a => transport dir
    TransportDirectionCommand transport_dir_shoot_command = new TransportDirectionCommand(m_transport_dir_subsystem, true);
    TransportDirectionCommand transport_dir_load_command = new TransportDirectionCommand(m_transport_dir_subsystem, false);
    LoadCommand load_command = new LoadCommand(m_left_loader_subsystem, m_right_loader_subsystem, m_transporter_subsystem);
    // StoreCommand store_command = new StoreCommand(new StorageSubsystem[] {m_right_storage_subsystem});

    m_driverController.a().toggleOnTrue(Commands.sequence(
      Commands.race(
        transport_dir_load_command
        // store_command
      ),
      Commands.parallel(  
        load_command
        // store_command
      ),
      transport_dir_shoot_command
    ));

    m_driverController.start().toggleOnTrue(m_right_storage_subsystem.run(()->m_right_storage_subsystem.setStore(StorageConstants.STORAGE_RPM)));
    m_driverController.back().toggleOnTrue(m_right_storage_subsystem.run(()->m_right_storage_subsystem.setStore(StorageConstants.REVERSE_RPM)));
    m_driverController.start().or(m_driverController.back()).onFalse(m_right_storage_subsystem.run(()->m_right_storage_subsystem.stop()));

    m_driverController.povUp().toggleOnTrue(m_left_storage_subsystem.run(()->m_left_storage_subsystem.setStore(StorageConstants.STORAGE_RPM)));
    m_driverController.povDown().toggleOnTrue(m_left_storage_subsystem.run(()->m_left_storage_subsystem.setStore(StorageConstants.REVERSE_RPM)));
    m_driverController.povUp().or(m_driverController.povDown()).onFalse(m_left_storage_subsystem.run(()->m_left_storage_subsystem.stop()));

    // pov => transport dir
    // m_driverController.povDown().onTrue(m_transport_dir_subsystem.run(()->{
    //   m_transport_dir_subsystem.setShoot();
    //   m_transport_dir_subsystem.setRunDirection();
    // }));
    // m_driverController.povUp() .onTrue(m_transport_dir_subsystem.run(()->{
    //   m_transport_dir_subsystem.setLoad();
    //   m_transport_dir_subsystem.setRunDirection();
    // }));
    // m_driverController.povDown().or(m_driverController.povUp()).onFalse(m_transport_dir_subsystem.run(()->m_transport_dir_subsystem.stopDirection()));

    // back => reset robot pos estimation
    m_driverController.back().toggleOnTrue(m_position_getter_subsystem.run(()->{
      m_position_getter_subsystem.resetRobotEstimation();
    }));

    // b => storage (WIP)
    // m_driverController.b().onTrue (m_left_storage_subsystem.run(()->m_left_storage_subsystem.setStore(0.4)));
    // m_driverController.b().onFalse(m_left_storage_subsystem.run(()->m_left_storage_subsystem.stop()));

    // x => transport
    m_driverController.x().toggleOnTrue(m_transporter_subsystem.transportCommand());
    m_driverController.x().toggleOnFalse(m_transporter_subsystem.stopTransportCommand());

    // AimCommand turnLeft  = new AimCommand(m_drive_subsystem, gyro, -90);
    // AimCommand turnRight = new AimCommand(m_drive_subsystem, gyro, +90);

    // drive movement (WIP)
    // turn left
    // turn right

    /*
    // drive movement
    m_drive_subsystem.setDefaultCommand(m_drive_subsystem.run(() -> m_drive_subsystem.drive(
      m_driverController.getLeftY(),
      m_driverController.getLeftX(),
      -m_driverController.getRightX()
    )));
    */

    // drive change speed modifier
    // m_driverController.leftBumper()
    //   .onTrue(m_drive_subsystem.runOnce(() -> m_drive_subsystem.decreaseSpeed()));
    // m_driverController.rightBumper()
    //   .onTrue(m_drive_subsystem.runOnce(() -> m_drive_subsystem.increaseSpeed()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(test_subsystem);
    return new ShootCommand(m_shooter_subsystem, m_transporter_subsystem);
  }
}
