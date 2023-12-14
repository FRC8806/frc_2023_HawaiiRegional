// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Catcher;

public class CatcherControl extends CommandBase {
  private final Catcher m_catcher;
  private final BooleanSupplier m_rb, m_lb;


  /** Creates a new ElevatorContriol. */
  public CatcherControl(Catcher catcher, BooleanSupplier rb, BooleanSupplier lb) {
    m_catcher = catcher;
    m_rb = rb;
    m_lb = lb;
    addRequirements(m_catcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_catcher.zeroNeckEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double neckSpeed = 
        m_rb.getAsBoolean() ? -0.5 :
        m_lb.getAsBoolean() ? 0.5 : 
        0;
    m_catcher.setNeckSpeed(-neckSpeed);
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
