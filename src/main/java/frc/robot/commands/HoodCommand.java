// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.ManualControls;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodCommand extends Command {
  private final Hood Hood;
  private final ManualControls controls;
  private final double targetAngle;
  /** Creates a new HoodCommand. */
  public HoodCommand(Hood hood, ManualControls controls, double targetAngle) {
    this.Hood = hood;
    this.controls = controls;
    this.targetAngle = targetAngle;
    addRequirements(hood);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Hood.setGoal(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.shootWithLimelight()) {
      Hood.runWithPose(); 
    }
    else if (controls.hoodManualUp()) {
      Hood.manualUp();
    }
    else if (controls.hoodManualDown()) {
      Hood.manualDown();
    }
    else {
      
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
