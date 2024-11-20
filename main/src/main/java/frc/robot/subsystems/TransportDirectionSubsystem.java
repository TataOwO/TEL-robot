// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransporterConstants;;

public class TransportDirectionSubsystem extends SubsystemBase {
  // hardware
  private final WPI_TalonSRX m_direction_motor;
  private DigitalOutput m_button;

  private boolean ready_load  = false;
  private boolean ready_shoot = true;
  
  private double direction_motor_speed = TransporterConstants.DIRECTION_ROTATION_SPEED;

  public TransportDirectionSubsystem() {
    m_direction_motor = new WPI_TalonSRX(TransporterConstants.DIRECTION_MOTOR_ID);

    // m_button = new DigitalOutput(TransporterConstants.TOP_BUTTON_PIN);
  }

  public void setRunDirection() {
    this.m_direction_motor.set(this.direction_motor_speed);
  }

  public void stopDirection() {
    this.m_direction_motor.set(0);
  }

  public void setShoot() {
    this.direction_motor_speed = TransporterConstants.DIRECTION_ROTATION_SPEED;
    this.ready_shoot = false;
    this.ready_shoot = false;
  }

  public void setLoad() {
    this.direction_motor_speed = -TransporterConstants.DIRECTION_ROTATION_SPEED;
    this.ready_shoot = false;
    this.ready_shoot = false;
  }

  public void setReady(boolean isShootAngle) {
    this.ready_shoot = isShootAngle;
    this.ready_load  = !isShootAngle;
  }

  public boolean isReadyShoot() {
    return this.ready_shoot;
  }

  public boolean isReadyLoad() {
    return this.ready_load;
  }

  public boolean getButton() {
    // return this.m_button.get();
    return false;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
