// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.ExtraShootCommand;
import frc.robot.commands.LoadManagerCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StoreCommand;
import frc.robot.subsystems.*;
import frc.robot.utility.CustomGyro;
import frc.robot.utility.CustomXboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // GRYO
  private final CustomGyro gyro = new CustomGyro(OperatorConstants.GYRO_PORT);

  // DRIVE
  private final MecanumDriveSubsystem m_drive_subsystem = new MecanumDriveSubsystem();

  // SHOOTER
  private final ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();

  // LOADERS
  private final LoaderSubsystem m_right_loader_subsystem = new LoaderSubsystem(LoaderConstants.loaderSide.RIGHT);
  private final LoaderSubsystem m_left_loader_subsystem = new LoaderSubsystem(LoaderConstants.loaderSide.LEFT);

  // TRANSPORT
  private final TransporterSubsystem m_transporter_subsystem = new TransporterSubsystem();
  private final TransportDirectionSubsystem m_transport_dir_subsystem = new TransportDirectionSubsystem();

  // STORAGE
  private final StorageSubsystem m_left_storage_subsystem  = new StorageSubsystem(StorageConstants.StorageSide.LEFT);
  private final StorageSubsystem m_right_storage_subsystem = new StorageSubsystem(StorageConstants.StorageSide.RIGHT);

  // LED STRIP
  private final LedStripSubsystem m_led_strip = new LedStripSubsystem(m_transporter_subsystem);

  // ULTRASONICS
  private final PositionGetterSubsystem m_position_getter_subsystem = new PositionGetterSubsystem(gyro, new UltrasonicSubsystem[] {
    new UltrasonicSubsystem(UltrasonicConstants.UltrasonicSide.FRONT),
    new UltrasonicSubsystem(UltrasonicConstants.UltrasonicSide.LEFT_FRONT),
    new UltrasonicSubsystem(UltrasonicConstants.UltrasonicSide.RIGHT_FRONT),
  });

  // CONTROLLER
  private CustomXboxController m_driverController =
      new CustomXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // TRANSPORT BUTTON TRIGGER
  Trigger transport_button = new Trigger(()->m_transporter_subsystem.getButton());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    gyro.reset();
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
    /******* AUTOMATIC FULL ACTION COMMANDS ********/

    // y => shooter
    m_driverController.y().and(m_driverController.leftTrigger(50).negate()).toggleOnTrue(new ShootCommand(m_shooter_subsystem, m_transporter_subsystem));
    m_driverController.y().and(m_driverController.leftTrigger(50))         .toggleOnTrue(new ExtraShootCommand(m_shooter_subsystem));

    // a => load disc
    m_driverController.a().toggleOnTrue(new LoadManagerCommand(m_transporter_subsystem, m_transport_dir_subsystem, m_left_loader_subsystem, m_right_loader_subsystem, m_left_storage_subsystem, m_right_storage_subsystem));

    // auto shoot dir
    // transport_button.toggleOnTrue(new TransportDirectionCommand(m_transport_dir_subsystem, true));

    /******* MANUAL COMMANDS ********/

    // pov up/down => right storage
    m_driverController.povUp().onTrue(new StoreCommand(m_right_storage_subsystem, true));
    m_driverController.povDown().onTrue(new StoreCommand(m_right_storage_subsystem, false));

    // pov left/right => left storage
    m_driverController.povLeft().onTrue(new StoreCommand(m_left_storage_subsystem, true));
    m_driverController.povRight().onTrue(new StoreCommand(m_left_storage_subsystem, false));

    // x => left loader
    m_driverController.x().and(m_driverController.leftTrigger(0.5).negate()).toggleOnTrue(m_left_loader_subsystem.run(()->{
      m_left_loader_subsystem.load();
    }));
    m_driverController.x().and(m_driverController.leftTrigger(0.5))         .toggleOnTrue(m_left_loader_subsystem.run(()->{
      m_left_loader_subsystem.reverse();
    }));
    m_driverController.x().toggleOnFalse(m_left_loader_subsystem.run(()->{
      m_left_loader_subsystem.stop();
    }));

    // b => right loader
    m_driverController.b().and(m_driverController.leftTrigger(0.5).negate()).toggleOnTrue(m_right_loader_subsystem.run(()->{
      m_right_loader_subsystem.load();
      m_transporter_subsystem.load();
    }));
    m_driverController.b().and(m_driverController.leftTrigger(0.5))         .toggleOnTrue(m_right_loader_subsystem.run(()->{
      m_right_loader_subsystem.reverse();
      m_transporter_subsystem.load();
    }));
    m_driverController.b().toggleOnFalse(m_right_loader_subsystem.run(()->{
      m_right_loader_subsystem.stop();
      m_transporter_subsystem.stop();
    }));

    // start back -> transport dir
    m_driverController.start().onTrue(m_transport_dir_subsystem.run(()->{
      m_transport_dir_subsystem.setLoad();
      m_transport_dir_subsystem.setRunDirection();
    }));
    m_driverController.back().onTrue(m_transport_dir_subsystem.run(()->{
      m_transport_dir_subsystem.setShoot();
      m_transport_dir_subsystem.setRunDirection();
    }));
    m_driverController.start().or(m_driverController.back()).toggleOnFalse(m_transport_dir_subsystem.run(()->{
      m_transport_dir_subsystem.stopDirection();
    }));

    /******* DRIVE COMMANDS ********/

    // AimCommand turnLeft  = new AimCommand(m_drive_subsystem, gyro, -90);
    // AimCommand turnRight = new AimCommand(m_drive_subsystem, gyro, +90);

    // drive movement (ABANDONED)
    // turn left
    // turn right

    // drive movement
    m_drive_subsystem.setDefaultCommand(m_drive_subsystem.run(() -> m_drive_subsystem.drive(
      m_driverController.getLeftY(),
      m_driverController.getLeftX(),
      -m_driverController.getRightX()
    )));

    // drive change speed modifier
    m_driverController.leftBumper()
      .onTrue(m_drive_subsystem.runOnce(() -> m_drive_subsystem.decreaseSpeed()));
    m_driverController.rightBumper()
      .onTrue(m_drive_subsystem.runOnce(() -> m_drive_subsystem.increaseSpeed()));
      
    // back => reset robot pos estimation
    m_driverController.rightStick().toggleOnTrue(m_position_getter_subsystem.run(()->{
      m_position_getter_subsystem.resetRobotEstimation();
    }));
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
