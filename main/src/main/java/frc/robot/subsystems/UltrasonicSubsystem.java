// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import javax.sound.midi.Soundbank;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.UltrasonicConstants;
import frc.robot.Constants.UltrasonicConstants.UltrasonicSide;;

public class UltrasonicSubsystem extends SubsystemBase {
  private final int trig_channel_id;
  private final int echo_channel_id;

  private final UltrasonicSide m_side;

  private final Ultrasonic ultrasonic;

  // position processing
  private final int MAX_READINGS_SIZE = 10;
  private final double MAX_DEVIATION = 100; // 10cm
  private final double MAX_VALID_DISTANCE = 5000; // 5meter
  private final double MIN_VALID_DISTANCE = 30;  // 3 cm
  private final double KALMAN_GAIN = 0.5;

  private final ArrayList<Double> recent_readings = new ArrayList<>();
  private int readings_size = 0;

  /** Creates a new UltrasonicSubsystem. */
  public UltrasonicSubsystem(UltrasonicSide side) {
    trig_channel_id = side.getTrigChannel();
    echo_channel_id = side.getEchoChannel();
    m_side = side;

    ultrasonic = new Ultrasonic(trig_channel_id, echo_channel_id);

    Ultrasonic.setAutomaticMode(true);
  }

  private double getProcessedRangeMM() {
    double raw_distance = getRangeMM();

    // check if distance out of range
    if (raw_distance < MIN_VALID_DISTANCE) return -1;
    if (raw_distance > MAX_VALID_DISTANCE) return -1;

    // add
    recent_readings.add(raw_distance);

    readings_size = recent_readings.size();

    // if array size exceeds, remove old
    if (readings_size > MAX_READINGS_SIZE) recent_readings.remove(0);
    else return raw_distance; // not enough data => return raw

    // first check
    double sum = 0;
    double average = 0;

    for (double reading : recent_readings) {
      sum += reading;
    }

    average = sum / MAX_READINGS_SIZE;

    // average with acceptable deviation
    sum = 0;
    int valid_readings = 0;

    for (double reading : recent_readings) {
      if (Math.abs(reading - average) > MAX_DEVIATION) continue;
      
      sum += reading;
      ++valid_readings;
    }

    // if we have more than 3 inputs with acceptable deviations
    if (valid_readings > 3) return sum / valid_readings;

    // otherwise, use simple kalman-inspired filter
    return average + this.KALMAN_GAIN*(raw_distance-average);
  }

  private double getRangeMM() {
    return this.ultrasonic.getRangeMM();
  }

  public UltrasonicSide getSide() {
    return this.m_side;
  }

  public double[] calculatePosition(double current_angle_radian) { // TODO: TEST THIS CODE
    // distance between ultrasonic and the center
    double base_x = -Math.sin(current_angle_radian);
    double base_y =  Math.cos(current_angle_radian);

    double sonic_angle_r = Math.toRadians(m_side.getBaseAngle()) + current_angle_radian;

    double distance = this.getProcessedRangeMM();

    if (distance == -1) return new double[] {-1};

    // distance between ultrasonic and object
    double distance_x = -Math.sin(sonic_angle_r) * distance;
    double distance_y =  Math.cos(sonic_angle_r) * distance;

    return new double[] {base_x+distance_x, -base_y-distance_y};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    String name = this.getSide().name();
    SmartDashboard.putBoolean(String.join(" ", name, "ultrasonic valid"), this.ultrasonic.isRangeValid());
    SmartDashboard.putNumber(String.join(" ", name, "ultrasonic range mm"), this.getRangeMM());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
