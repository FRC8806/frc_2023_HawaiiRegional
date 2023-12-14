// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Tools;

public class Catcher extends SubsystemBase {
  private DoubleSolenoid cylinder = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, Constants.CatcherConstants.CYLINDER_FORWARD_ID, Constants.CatcherConstants.CYLINDER_REVERSE_ID);
  private Solenoid ledPurple = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CatcherConstants.LED_PURPLE_PORT);
  private Solenoid ledYello = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CatcherConstants.LED_YELLO_PORT);
  private TalonFX motorNeck = new TalonFX(Constants.CatcherConstants.MOTOR_NECK_ID);
  // private TalonFX motorHead = new TalonFX(Constants.CatcherConstants.MOTOR_HEAD_ID);
  private Boolean isCatched = true;
  private Boolean isCone = false;

  public Catcher() {
    ledYello.set(true);
    ledPurple.set(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IsCatched", isCatched);
    SmartDashboard.putNumber("Neck Angle", getNeckAngle());
    SmartDashboard.putBoolean("CONE/CUBE", isCone);

  }

  // /**
  //  * 
  //  * @param speed Turn as CCW when speed > 0.
  //  */
  // public void setMotorHeadSpeed(double speed) {
  //   motorHead.set(ControlMode.PercentOutput, speed);
  // }

  // public double getMotorHeadAngle() {
  //   return motorHead.getSelectedSensorPosition();
  // }

  /**
   * 
   * @param speed Turn up when speed < 0.
   */
  public void setNeckSpeed(double speed) {
    speed = new Tools().setOnLimit(
        speed, getNeckAngle(), 
        Constants.CatcherConstants.N_NECK_LOW_LIMIT, 
        Constants.CatcherConstants.N_NECK_HIGH_LIMIT, 
        0.05, 
        0.05);
    motorNeck.set(ControlMode.PercentOutput, speed);
  }

  public double getNeckAngle() {
    return motorNeck.getSelectedSensorPosition();
  }

  public void zeroNeckEncoder() {
    motorNeck.setSelectedSensorPosition(0);
  }

  public void catchOn() {
    cylinder.set(Value.kForward);
    isCatched = true;
  }

  public void release() {
    cylinder.set(Value.kReverse);
    isCatched = false;
  }

  public void setLedYello(boolean on) {
    ledYello.set(on);
    isCone = on;
  }

  public void setLedPurple(boolean on) {
    ledPurple.set(on);
    isCone = !on;
  }

  public void setNeckOutOfLimit(double speed) {
    motorNeck.set(ControlMode.PercentOutput, speed);
  }
}
