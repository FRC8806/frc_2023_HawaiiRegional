// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Catcher;

public class TurnBack extends CommandBase {
  private final Arm m_arm;
  private final Catcher m_catcher;
  /** Creates a new TurnBack. */
  public TurnBack(Arm arm, Catcher catcher) {
    m_catcher = catcher;
    m_arm = arm;
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
    double elevatorSpeed = (Constants.NodeConstants.N_TURN_BACK[0] - m_arm.getElavatorPosition())/50000;
    double armSpeed = (Constants.NodeConstants.N_TURN_BACK[1] - m_arm.getArmPosition())/50000;
    double neckSpeed = (Constants.NodeConstants.N_TURN_BACK[2] - m_catcher.getNeckAngle())/10000;
    neckSpeed = neckSpeed > 1 ? 1 : neckSpeed;
    elevatorSpeed = elevatorSpeed > 0 ? elevatorSpeed*1.5 : elevatorSpeed;
    m_arm.setElavatorSpeed(elevatorSpeed);
    m_arm.setArmSpeed(armSpeed);
    m_catcher.setNeckSpeed(neckSpeed*0.6);
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
