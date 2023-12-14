// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Tools;

public class Arm extends SubsystemBase {
  private final TalonFX motorLeft = new TalonFX(Constants.ArmConstants.MOTOR_LEFT_ID);
  private final TalonFX motorRight = new TalonFX(Constants.ArmConstants.MOTOR_RIGHT_ID);
  private final TalonFX motorMiddle = new TalonFX(Constants.ArmConstants.MOTOR_MIDDLE_ID);
  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("EPosition", getElavatorPosition());
    SmartDashboard.putNumber("ArmPosition", getArmPosition());
  }
  /**
   * 
   * @param speed Elevator up when speed > 0.
   */
  public void setElavatorSpeed(double speed) {
    speed = new Tools().setOnLimit(
        speed, getElavatorPosition(), 
        Constants.ArmConstants.N_ELEVATOR_LOW_LIMIT, 
        Constants.ArmConstants.N_ELEVATOR_HIGH_LIMIT, 
        0.15, 
        0.22);
    speed = speed > 0 ? speed : speed;
    motorLeft.set(ControlMode.PercentOutput, speed);
    motorRight.set(ControlMode.PercentOutput, -speed);
  }
  /**
   * Arm down when speed > 0.
   * @param speed
   */
  public void setArmSpeed(double speed) {
    speed = new Tools().setOnLimit(
        speed, 
        getArmPosition(), 
        Constants.ArmConstants.N_ARM_LOW_LIMIT, 
        Constants.ArmConstants.N_ARM_HIGH_LIMIT, 
        0.08, 0.2);
    speed = Math.abs(speed) > 0.1 ? speed : 0;
    motorMiddle.set(ControlMode.PercentOutput, speed);
  }

  public double getElavatorPosition() {
    return (motorLeft.getSelectedSensorPosition() - motorRight.getSelectedSensorPosition())/2;
  }

  public double getArmPosition() {
    return motorMiddle.getSelectedSensorPosition();
  }

  public void zeroArmEncoder() {
    motorMiddle.setSelectedSensorPosition(0);
  }

  public void zeroElevatorEncoder() {
    motorLeft.setSelectedSensorPosition(0);
    motorRight.setSelectedSensorPosition(0);
  }

  public void setElavatorOutOfLimit(double speed) {
    motorLeft.set(ControlMode.PercentOutput, speed);
    motorRight.set(ControlMode.PercentOutput, -speed);
  }

  public void setArmOutOfLimit(double speed) {
    motorMiddle.set(ControlMode.PercentOutput, speed);
  }
}
