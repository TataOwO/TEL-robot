// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooter;
  private final TalonFXConfiguration configs = new TalonFXConfiguration();

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private StatusCode status = StatusCode.StatusCodeNotInitialized;

  public ShooterSubsystem() {
    configs.Slot0.kP = 0.1;
    configs.Slot0.kI = 0.01;
    configs.Slot0.kD = 0;

    shooter = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);

    for (int i=0; i<5; i++) {
      status = shooter.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) System.out.println("An error occured at shooter subsys: " + status.toString());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(
        () -> {
          
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

  /**
   * 
   * @param speed rotation per minutes, but not precise
   */
  private void shoot(double speed) {
    shooter.setControl(m_velocityVoltage.withVelocity(speed));
  }

  private void stop() {
    shooter.setControl(m_brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
