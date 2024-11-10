// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootCommand;

public class TestSubsystem extends SubsystemBase {
  private final Timer timer = new Timer();
  private final ShooterSubsystem shooter_subsystem;
  private final TransporterSubsystem transport_subsystem;
  
  private boolean ran = false;
  private ShootCommand shootCommand;

  /** Creates a new ExampleSubsystem. */
  public TestSubsystem(ShooterSubsystem shooter_subsystem, TransporterSubsystem transport_subsystem) {
    this.shooter_subsystem = shooter_subsystem;
    this.transport_subsystem = transport_subsystem;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (ran) return;

    if (timer.get() > 5) {
      this.shootCommand = new ShootCommand(shooter_subsystem, transport_subsystem);
      ran = true;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
