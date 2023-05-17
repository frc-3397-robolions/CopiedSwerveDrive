// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoDriveToPose;
import frc.robot.commands.BasicAutoDrive;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.SpinCommand;
import frc.robot.commands.DriveWithJoysticks.JoystickMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import static frc.robot.Constants.*;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Pose2d autoDriveTarget = new Pose2d(4.0, 2.0, new Rotation2d());

  // Subsystems
  private Drive drive;

  // OI objects
  private XboxController driverController = new XboxController(0);
  private XboxController operatorController = new XboxController(1);
  private GenericHID keyboard = new GenericHID(2);
  private boolean isFieldRelative = true;

  // Choosers
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final LoggedDashboardChooser<JoystickMode> joystickModeChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private final LoggedDashboardChooser<Double> demoLinearSpeedLimitChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private final LoggedDashboardChooser<Double> demoAngularSpeedLimitChooser =
      new LoggedDashboardChooser<>("Angular Speed Limit");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2022S:
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOSparkMAX(0),
                  new ModuleIOSparkMAX(1),
                  new ModuleIOSparkMAX(2),
                  new ModuleIOSparkMAX(3));
          break;
        case ROBOT_SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          break;
        default:
          break;
      }
    }

    // Instantiate missing subsystems
    drive =
        drive != null
            ? drive
            : new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

    // Set up auto routines
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Drive Straight", new BasicAutoDrive(drive, new ChassisSpeeds(3,0,0)));

    // Set up choosers
    joystickModeChooser.addDefaultOption("Standard", JoystickMode.Standard);
    joystickModeChooser.addOption("Tank", JoystickMode.Tank);
    demoLinearSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    demoLinearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoLinearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoLinearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    demoAngularSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    demoAngularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoAngularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoAngularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.", AlertType.INFO)
          .set(true);
    }

    configureButtonBindings();
    configureAutoCommands();
  }
  private void configureAutoCommands(){
    AUTO_EVENT_MAP.put("shoot", new PrintCommand("*shoots*"));
    AUTO_EVENT_MAP.put("pickup", new BasicAutoDrive(drive, new ChassisSpeeds(0, 0, 0.5*drive.getMaxAngularSpeedRadPerSec())).withTimeout(2));
    AUTO_EVENT_MAP.put("drop", new PrintCommand("*drops*"));

    List<PathPlannerTrajectory> leftAuto = PathPlanner.loadPathGroup("2CubeAutoLeft", AUTO_MAX_VEL, AUTO_MAX_ACCEL);
    List<PathPlannerTrajectory> midAuto = PathPlanner.loadPathGroup("2CubeAutoMiddle", AUTO_MAX_VEL, AUTO_MAX_ACCEL);
    List<PathPlannerTrajectory> rightAuto = PathPlanner.loadPathGroup("2CubeAutoRight", AUTO_MAX_VEL, AUTO_MAX_ACCEL);

    Logger.getInstance().recordOutput("Path", leftAuto.get(0));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drive::getPose, // Pose2d supplier
    drive::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    new PIDConstants(1.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    drive::runVelocity, // Module states consumer used to output to the drive subsystem
    AUTO_EVENT_MAP,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    drive //The drive subsystem. Used to properly set the requirements of path following commands
  );
    Command TwoCubeAutoLeft = new InstantCommand(()->drive.setPose(leftAuto.get(0).getInitialHolonomicPose())).andThen(autoBuilder.fullAuto(leftAuto));
    autoChooser.addDefaultOption("TwoCubeAutoLeft", TwoCubeAutoLeft);

    Command TwoCubeAutoMid = autoBuilder.fullAuto(midAuto);
    autoChooser.addOption("TwoCubeAutoMid", TwoCubeAutoMid);

    Command TwoCubeAutoRight = autoBuilder.fullAuto(rightAuto);
    autoChooser.addOption("TwoCubeAutoRight", TwoCubeAutoRight);
  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driving controls
    new Trigger(driverController::getStartButton)
        .or(new Trigger(driverController::getBackButton))
        .onTrue(
            new InstantCommand(
                    () -> {
                      isFieldRelative = !isFieldRelative;
                      SmartDashboard.putBoolean("Field Relative", isFieldRelative);
                    })
                .ignoringDisable(true));
    SmartDashboard.putBoolean("Field Relative", isFieldRelative);
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> !isFieldRelative,
            () -> joystickModeChooser.get(),
            () -> demoLinearSpeedLimitChooser.get(),
            () -> demoAngularSpeedLimitChooser.get(),
            () -> driverController.getRightTriggerAxis()));

    // Reset gyro command
    Command resetGyroCommand =
        new InstantCommand(
                () -> {
                  drive.setPose(autoDriveTarget);
                },
                drive)
            .ignoringDisable(true);
    Command rumbleCommand =
        new StartEndCommand(
            () -> driverController.setRumble(RumbleType.kRightRumble, 0.5),
            () -> driverController.setRumble(RumbleType.kRightRumble, 0.0)) {
          @Override
          public boolean runsWhenDisabled() {
            return true;
          }
        }.withTimeout(0.2);
    new Trigger(driverController::getLeftBumper)
        .and(new Trigger(driverController::getRightBumper))
        .onTrue(resetGyroCommand)
        .onTrue(rumbleCommand);
    
    Command driveToOrigin =
        new AutoDriveToPose(drive, new Pose2d());

    new Trigger(() -> keyboard.getRawButton(1))
        .onTrue(driveToOrigin);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
