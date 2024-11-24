// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_shooter;
  private final WPI_VictorSPX m_shooter_support;
  private final TalonFXConfiguration shooter_configs = new TalonFXConfiguration();

  private final VelocityVoltage m_velocity_voltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private final StatusSignal<Double> shooter_velocity;

  private StatusCode shooter_status = StatusCode.StatusCodeNotInitialized;

  private Map<String, Boolean> grid = new HashMap<>() {{
    put("TL goal", false);
    put("TC goal", false);
    put("TR goal", false);
    put("CL goal", false);
    put("CC goal", true);
    put("CR goal", false);
    put("BL goal", false);
    put("BC goal", false);
    put("BR goal", false);
  }};

  public ShooterSubsystem() {
    shooter_configs.Slot0.kP = 0.1;
    shooter_configs.Slot0.kI = 0.01;
    shooter_configs.Slot0.kD = 0;

    m_shooter = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
    m_shooter_support = new WPI_VictorSPX(ShooterConstants.SHOOTER_SUPPORT_MOTOR_ID);

    for (int i=0; i<5; i++) {
      shooter_status = m_shooter.getConfigurator().apply(shooter_configs);
      if (shooter_status.isOK()) break;
    }
    if (!shooter_status.isOK()) System.out.println("An error occured at shooter subsystem: " + shooter_status.toString());

    shooter_velocity = m_shooter.getVelocity();
    
    for (Entry<String, Boolean> key: grid.entrySet()) {
      SmartDashboard.putBoolean(key.getKey(), key.getValue());
    }
  }

  /**
   * 
   * @param RPS rotation per second
   */
  public Command shootCommand(double RPS, double support_speed) {
    return this.runOnce(shootRunnable(RPS, support_speed));
  }

  public Command stopCommand() {
    return this.runOnce(stopRunnable());
  }

  /**
   * 
   * @param RPS rotation per second
   */
  private Runnable shootRunnable(double RPS, double support_speed) {
    return () -> shoot(RPS, support_speed);
  }

  private Runnable stopRunnable() {
    return () -> stop();
  }

  /**
   * 
   * @param RPS rotation per second 0~120
   * @param support_speed current 0.0~1.0
   */
  public void shoot(double RPS, double support_speed) {
    m_shooter.setControl(m_velocity_voltage.withVelocity(RPS));
    m_shooter_support.set(support_speed);
  }

  public void stop() {
    m_shooter.setControl(m_brake);
    m_shooter_support.set(0);
  }

  @Override
  public void periodic() {
    shooter_velocity.refresh();

    for (Entry<String, Boolean> key: grid.entrySet()) {
      String name = key.getKey();
      boolean val = key.getValue();

      boolean new_val = SmartDashboard.getBoolean(name, val);

      if (val && !new_val) {
        SmartDashboard.putBoolean(name, true);
      } else if (new_val && !val) {
        for (Entry<String, Boolean> n_key: grid.entrySet()) {
          String n_name = n_key.getKey();
          if (grid.get(n_name)) {
            grid.put(n_name, false);
            SmartDashboard.putBoolean(n_name, false);
            break;
          }
        }
        grid.put(name, true);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    shooter_velocity.refresh();
  }
}
