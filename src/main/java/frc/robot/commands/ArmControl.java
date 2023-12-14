// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmControl extends CommandBase {
  private final Arm m_arm;
  private final DoubleSupplier m_ry, m_rt, m_lt;
  
  /** Creates a new armControl. */
  public ArmControl(Arm arm, DoubleSupplier ry, DoubleSupplier rt, DoubleSupplier lt) {
    this.m_arm = arm;
    m_ry = ry;
    m_rt = rt;
    m_lt = lt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_arm.zeroArmEncoder();
    // m_arm.zeroElevatorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = m_rt.getAsDouble() - m_lt.getAsDouble();
    double elevatorSpeed = -m_ry.getAsDouble();
    m_arm.setArmSpeed(armSpeed);
    m_arm.setElavatorSpeed(elevatorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
