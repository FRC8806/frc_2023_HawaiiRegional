// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Catcher;

public class AutoNeckDown extends CommandBase {
  private final Catcher m_catcher;
  /** Creates a new AutoNeckDown. */
  public AutoNeckDown(Catcher catcher) {
    m_catcher = catcher;
    addRequirements(m_catcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double neckSpeed = (Constants.NodeConstants.N_AUTO_NECK_DOWN - m_catcher.getNeckAngle())/10000;
    m_catcher.setNeckSpeed(neckSpeed*0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_catcher.release();
    m_catcher.setNeckSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_catcher.getNeckAngle() > 130000;
  }
}
