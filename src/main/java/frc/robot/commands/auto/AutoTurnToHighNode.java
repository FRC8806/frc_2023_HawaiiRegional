// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Catcher;

public class AutoTurnToHighNode extends CommandBase {
  private final Arm m_arm;
  private final Catcher m_catcher;
  /** Creates a new TurnOnHighState. */
  public AutoTurnToHighNode(Arm arm, Catcher catcher) {
    m_catcher = catcher;
    m_arm = arm;
    addRequirements(m_arm);
    addRequirements(m_catcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorSpeed = (Constants.NodeConstants.N_HIGH_CONE_NODE[0] - m_arm.getElavatorPosition())/50000;
    double armSpeed = (Constants.NodeConstants.N_HIGH_CONE_NODE[1] + 100000 - m_arm.getArmPosition())/50000;
    double neckSpeed = (Constants.NodeConstants.N_HIGH_CONE_NODE[2] - m_catcher.getNeckAngle())/30000;
    neckSpeed = neckSpeed > 1 ? 1 : neckSpeed;
    elevatorSpeed = elevatorSpeed > 0 ? elevatorSpeed*1.5 : elevatorSpeed;
    // new Tools().range(neckSpeed, 0.5, -0.5);
    m_arm.setElavatorSpeed(elevatorSpeed);
    m_arm.setArmSpeed(armSpeed);
    m_catcher.setNeckSpeed(neckSpeed*0.5);
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
    return m_arm.getArmPosition() > Constants.NodeConstants.N_HIGH_CONE_NODE[1] + 100000 - 5000 && 
        m_arm.getElavatorPosition() > Constants.NodeConstants.N_HIGH_CONE_NODE[0] - 5000 && 
        m_catcher.getNeckAngle() > Constants.NodeConstants.N_HIGH_CONE_NODE[2] - 5000;
  }
}
