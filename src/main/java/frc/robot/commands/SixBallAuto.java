// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.trajectory.Waypoint;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBallAuto extends SequentialCommandGroup {
  public static Pose2d centerHangarPosition = new Pose2d(new Translation2d(
      (FieldConstants.fieldWidth - (FieldConstants.hangarWidth / 2)),
      FieldConstants.hangarLength / 2), new Rotation2d());

  public static Pose2d rightBeforeCargoB = FieldConstants.cargoB.transformBy(
      new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(0)));

  public static Pose2d endPosition = FieldConstants.cargoB;

  /** Creates a new SixBallAuto. */
  public SixBallAuto(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ThreeCargoAuto(drive),
        // spawn to cargoG
        new AutoDrive(drive,
            List.of(Waypoint.fromHolonomicPose(ThreeCargoAuto.cargoDPosition),
                Waypoint.fromHolonomicPose(FiveCargoAuto.cargoGPosition))),
        // cargoG to Terminal
        new AutoDrive(drive,
            List.of(Waypoint.fromHolonomicPose(FiveCargoAuto.cargoGPosition),
                Waypoint.fromHolonomicPose(centerHangarPosition))),
        // cargoG mario kart around hangar to right before cargoB
        new AutoDrive(drive,
            List.of(Waypoint.fromHolonomicPose(FiveCargoAuto.cargoGPosition),
                Waypoint.fromHolonomicPose(centerHangarPosition),
                Waypoint.fromHolonomicPose(rightBeforeCargoB))),
        new AutoDrive(drive,
            // right before cargoB to cargoB
            List.of(Waypoint.fromHolonomicPose(rightBeforeCargoB),
                Waypoint.fromHolonomicPose(endPosition))));

  }

  public static Pose2d calcAimedPose(Pose2d pose) {
    Translation2d vehicleToCenter =
        FieldConstants.hubCenter.minus(pose.getTranslation());
    Rotation2d targetRotation =
        new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
    targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
    return new Pose2d(pose.getTranslation(), targetRotation);
  }
}
