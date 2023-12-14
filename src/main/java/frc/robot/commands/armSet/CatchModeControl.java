// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armSet;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Catcher;

public class CatchModeControl extends CommandBase {
  private final Arm m_arm;
  private final Catcher m_catcher;
  private final IntSupplier m_pov;
  private double distance = 20;
  /** Creates a new CatchModeControl. */
  public CatchModeControl(Arm arm, Catcher catcher, IntSupplier pov) {
    m_arm = arm;
    m_catcher = catcher;
    m_pov = pov;
    addRequirements(m_arm);
    addRequirements(m_catcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance >= 20 && m_pov.getAsInt() == 270) distance -= 1;
    if(distance <= 80 && m_pov.getAsInt() == 90) distance += 1;    
    double targetArm = 104443 - 113 * distance + 58.7 * distance * distance;
    double targetNeck = 228249 - 662 * distance - 4.99 * distance * distance;
    // SmartDashboard.putNumber("ta", targetArm);
    // SmartDashboard.putNumber("tn", targetNeck);
    // SmartDashboard.putNumber("dis", distance);

    double armSpeed = (targetArm - m_arm.getArmPosition())/50000;
    double neckSpeed = (targetNeck - m_catcher.getNeckAngle())/60000;
    m_arm.setArmSpeed(armSpeed);
    m_catcher.setNeckSpeed(neckSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setArmSpeed(0);
    m_arm.setElavatorSpeed(0);
    m_catcher.setNeckSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
