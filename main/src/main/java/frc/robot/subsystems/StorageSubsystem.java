// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants.StorageSide;

public class StorageSubsystem extends SubsystemBase {
  private final DigitalOutput m_button;
  private final CANSparkMax m_motor;
  
  private final SparkPIDController motor_pid;

  private final StorageSide side;

  /** Creates a new StorageSubsystem. */
  public StorageSubsystem(StorageSide side) {
    this.side = side;

    m_motor  = new CANSparkMax(side.getMotorId(), MotorType.kBrushless);

    motor_pid = m_motor.getPIDController();

    // motor_pid.setP(0.005);
    // motor_pid.setI(0);
    // motor_pid.setD(0);
    // motor_pid.setFF(0);
    m_button = new DigitalOutput(side.getButtonId());
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

  public void setStore(double speed) {
    // motor_pid.setReference(speed, ControlType.kCurrent);
    m_motor.set(speed);
  }

  public void stop() {
    // motor_pid.setReference(0, ControlType.kCurrent);
    m_motor.set(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean buttonPressed() {
    return m_button.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    String button_status_key = String.join(" ", "storage", this.side.name(), "button");

    SmartDashboard.putBoolean(button_status_key, this.buttonPressed());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
