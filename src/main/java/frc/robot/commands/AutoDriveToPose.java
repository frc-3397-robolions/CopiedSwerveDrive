// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2_StickyFaults;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.trajectory.CustomHolonomicDriveController;

public class AutoDriveToPose extends CommandBase {
  /** Creates a new AutoDriveToPose. */
  private CustomHolonomicDriveController controller;
  private Drive drive;

  private Pose2d desiredPose;
  public AutoDriveToPose(Drive drive, Pose2d desiredPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.desiredPose = desiredPose;
    addRequirements(drive);

    controller = new CustomHolonomicDriveController(
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0));
    controller.setTolerance(new Pose2d(new Translation2d(0.1,0.1),new Rotation2d(0.1)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runVelocity(controller.calculate(
      drive.getPose(),
      desiredPose,
      1,
      desiredPose.getRotation(),
      0.1));
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atReference();
  }
}
