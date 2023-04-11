// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class SpinCommand extends CommandBase {
  /** Creates a new SpinCommand. */
  Drive drive;
  double startpos;
  public SpinCommand(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive=drive;
    addRequirements(drive);
    startpos=drive.getRotation().getRotations();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.runVelocity(new ChassisSpeeds(0,0,drive.getMaxAngularSpeedRadPerSec()*0.5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getRotation().getRotations()-startpos>=1;
  }
}
