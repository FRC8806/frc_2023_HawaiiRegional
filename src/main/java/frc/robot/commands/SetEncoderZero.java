// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Catcher;

public class SetEncoderZero extends CommandBase {
  private final Catcher m_catcher;
  private final Arm m_arm;
  private final DoubleSupplier m_rt, m_lt, m_ry;
  private final BooleanSupplier m_rb, m_lb;
  /** Creates a new setEncoderZero. */
  public SetEncoderZero(Catcher catcher, Arm arm, DoubleSupplier rt, DoubleSupplier lt, DoubleSupplier ry, BooleanSupplier rb, BooleanSupplier lb) {
    m_catcher = catcher;
    m_arm = arm;
    m_rt = rt;
    m_lt = lt;
    m_ry = ry;
    m_rb = rb;
    m_lb = lb;
    addRequirements(m_catcher);
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("is resetCommand on", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setArmOutOfLimit((m_rt.getAsDouble() - m_lt.getAsDouble())*0.3);
    m_arm.setElavatorOutOfLimit(-m_ry.getAsDouble()*0.3);
    m_catcher.setNeckOutOfLimit(
        m_rb.getAsBoolean() ? 0.1 :
        m_lb.getAsBoolean() ? -0.1 :
        0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.zeroArmEncoder();
    m_arm.zeroElevatorEncoder();
    m_catcher.zeroNeckEncoder();
    SmartDashboard.putBoolean("is resetCommand on", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
