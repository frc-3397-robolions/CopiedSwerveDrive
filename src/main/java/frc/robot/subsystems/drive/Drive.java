// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhotonCameraWrapper;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Drive extends SubsystemBase {
  private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
  // switch to coast when disabling

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final ModuleIO[] moduleIOs = new ModuleIO[4]; // FL, FR, BL, BR
  private final ModuleIOInputsAutoLogged[] moduleInputs =
      new ModuleIOInputsAutoLogged[] {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged()
      };

  private final double maxLinearSpeed;
  private final double maxAngularSpeed;
  private final double wheelRadius;
  private final double trackWidthX;
  private final double trackWidthY;

  private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp");
  private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd");
  private final LoggedTunableNumber driveKs = new LoggedTunableNumber("Drive/DriveKs");
  private final LoggedTunableNumber driveKv = new LoggedTunableNumber("Drive/DriveKv");

  private final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/TurnKp");
  private final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/TurnKd");

  private final SwerveDriveKinematics kinematics;
  private SimpleMotorFeedforward driveFeedforward;
  private final PIDController[] driveFeedback = new PIDController[4];
  private final PIDController[] turnFeedback = new PIDController[4];

  private Pose2d odometryPose = new Pose2d();
  private Translation2d fieldVelocity = new Translation2d();
  private double[] lastModulePositionsRad = new double[] {0.0, 0.0, 0.0, 0.0};
  private double lastGyroPosRad = 0.0;
  private boolean brakeMode = false;

  private DriveMode driveMode = DriveMode.NORMAL;
  private ChassisSpeeds closedLoopSetpoint = new ChassisSpeeds();

  public PhotonCameraWrapper pcw = new PhotonCameraWrapper();
  private SwerveDrivePoseEstimator m_PoseEstimator;
  /** Creates a new Drive. */
  public Drive(GyroIO gyroIO,ModuleIO flModuleIO,ModuleIO frModuleIO,ModuleIO blModuleIO,ModuleIO brModuleIO) {

    this.gyroIO = gyroIO;
    moduleIOs[0] = flModuleIO;
    moduleIOs[1] = frModuleIO;
    moduleIOs[2] = blModuleIO;
    moduleIOs[3] = brModuleIO;
    
    maxLinearSpeed = Units.feetToMeters(14.5);
    wheelRadius = Units.inchesToMeters(2.0);
    trackWidthX = Units.inchesToMeters(25.0);
    trackWidthY = Units.inchesToMeters(24.0);

    //Assign robot constants based on mode
    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        driveKp.initDefault(0.1);
        driveKd.initDefault(0.0);
        driveKs.initDefault(0.12349);
        driveKv.initDefault(0.13477);

        turnKp.initDefault(10.0);
        turnKd.initDefault(0.0);
        break;
      case ROBOT_SIMBOT:
        driveKp.initDefault(0.9);
        driveKd.initDefault(0.0);
        driveKs.initDefault(0.116970);
        driveKv.initDefault(0.133240);

        turnKp.initDefault(23.0);
        turnKd.initDefault(0.0);
        break;
      default:
        driveKp.initDefault(0.0);
        driveKd.initDefault(0.0);
        driveKs.initDefault(0.0);
        driveKv.initDefault(0.0);

        turnKp.initDefault(0.0);
        turnKd.initDefault(0.0);
        break;
    }

    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    //Make unique PID controllers for each module. May or may not be neccesary
    for (int i = 0; i < 4; i++) {
      driveFeedback[i] =
          new PIDController(driveKp.get(), 0.0, driveKd.get(), Constants.loopPeriodSecs);
      turnFeedback[i] =
          new PIDController(turnKp.get(), 0.0, turnKd.get(), Constants.loopPeriodSecs);
      turnFeedback[i].enableContinuousInput(-Math.PI, Math.PI);
    }
    m_PoseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      Rotation2d.fromRadians(gyroInputs.positionRad),
      new SwerveModulePosition[] {
        moduleInputs[0].pos,
        moduleInputs[1].pos,
        moduleInputs[2].pos,
        moduleInputs[3].pos
      },
      odometryPose
      ); 

    // Calculate max angular speed
    maxAngularSpeed =
        maxLinearSpeed
            / Arrays.stream(getModuleTranslations())
                .map(translation -> translation.getNorm())
                .max(Double::compare)
                .get();
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
      Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i), moduleInputs[i]);
    }
    // Update angle measurements
    Rotation2d[] turnPositions = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      turnPositions[i] = new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
    }
    // Update objects based on TunableNumbers
    updateTunables();

    // Disable output while disabled
    if (DriverStation.isDisabled()) {
      
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(0.0);
        moduleIOs[i].setDriveVoltage(0.0);
      }
    } else {
      switch (driveMode) {
/*Run based on closedLoopSetpoint*/case NORMAL:
          // In normal mode, run the controllers for turning and driving based on the current
          // setpoint
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(closedLoopSetpoint);
          SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed);

          // If stationary, go to last state
          boolean isStationary =
              Math.abs(closedLoopSetpoint.vxMetersPerSecond) < 1e-3
                  && Math.abs(closedLoopSetpoint.vyMetersPerSecond) < 1e-3
                  && Math.abs(closedLoopSetpoint.omegaRadiansPerSecond) < 1e-3;

          SwerveModuleState[] setpointStatesOptimized =
              new SwerveModuleState[] {null, null, null, null};
          for (int i = 0; i < 4; i++) {
            // Run turn controller
            setpointStatesOptimized[i] =
                SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
            if (isStationary) {
              moduleIOs[i].setTurnVoltage(0.0);
            } else {
              moduleIOs[i].setTurnVoltage(
                  turnFeedback[i].calculate(
                      turnPositions[i].getRadians(),
                      setpointStatesOptimized[i].angle.getRadians()));
            }

            // Update velocity based on turn error
            setpointStatesOptimized[i].speedMetersPerSecond *=
                Math.cos(turnFeedback[i].getPositionError());

            // Run drive controller
            double velocityRadPerSec =
                setpointStatesOptimized[i].speedMetersPerSecond / wheelRadius;
            moduleIOs[i].setDriveVoltage(
                driveFeedforward.calculate(velocityRadPerSec)
                    + driveFeedback[i].calculate(
                        moduleInputs[i].driveVelocityRadPerSec, velocityRadPerSec));

            // Log individual setpoints
            Logger.getInstance()
                .recordOutput(
                    "SwerveSetpointValues/Drive/" + Integer.toString(i), velocityRadPerSec);
            Logger.getInstance()
                .recordOutput(
                    "SwerveSetpointValues/Turn/" + Integer.toString(i),
                    setpointStatesOptimized[i].angle.getRadians());
          }

          // Log all module setpoints
          Logger.getInstance().recordOutput("SwerveModuleStates/Setpoints", setpointStates);
          Logger.getInstance()
              .recordOutput("SwerveModuleStates/SetpointsOptimized", setpointStatesOptimized);
          break;
 
          
/*Go to X Stance*/  case X:
          for (int i = 0; i < 4; i++) {
            Rotation2d targetRotation = GeomUtil.direction(getModuleTranslations()[i]);
            Rotation2d currentRotation = turnPositions[i];
            if (Math.abs(targetRotation.minus(currentRotation).getDegrees()) > 90.0) {
              targetRotation = targetRotation.minus(Rotation2d.fromDegrees(180.0));
            }
            moduleIOs[i].setTurnVoltage(
                turnFeedback[i].calculate(
                    currentRotation.getRadians(), targetRotation.getRadians()));
            moduleIOs[i].setDriveVoltage(0.0);
          }
          break;
      }
    }

    

    // Update field velocity
    SwerveModuleState[] measuredStates = new SwerveModuleState[] {null, null, null, null};
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = moduleInputs[i].state;
    }
    ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
    fieldVelocity =
        new Translation2d(chassisState.vxMetersPerSecond, chassisState.vyMetersPerSecond)
            .rotateBy(getRotation());
    if(Constants.getRobot()==RobotType.ROBOT_SIMBOT){
      //If in sim mode, make up gyro pos from chassis states
      fieldVelocity = new Translation2d(chassisState.vxMetersPerSecond, chassisState.vyMetersPerSecond)
                        .rotateBy(Rotation2d.fromRadians(chassisState.omegaRadiansPerSecond));
      gyroInputs.velocityRadPerSec = chassisState.omegaRadiansPerSecond;
      gyroInputs.positionRad+=chassisState.omegaRadiansPerSecond;
      gyroInputs.rotation2d=Rotation2d.fromRadians(gyroInputs.positionRad);
    }

    // Update odometry
    updateOdometry(moduleInputs); 

    // Log measured states
    Logger.getInstance().recordOutput("SwerveModuleStates/Measured", measuredStates);

    // Log odometry pose
    Logger.getInstance().recordOutput("Odometry/Robot", odometryPose);

    // Enable/disable brake mode
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(true);
          moduleIOs[i].setDriveBrakeMode(true);
        }
      }
    } else {
      boolean stillMoving = false;
      for (int i = 0; i < 4; i++) {
        if (Math.abs(moduleInputs[i].driveVelocityRadPerSec * wheelRadius)
            > maxCoastVelocityMetersPerSec) {
          stillMoving = true;
        }
      }

      if (brakeMode && !stillMoving) {
        brakeMode = false;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(false);
          moduleIOs[i].setDriveBrakeMode(false);
        }
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    driveMode = DriveMode.NORMAL;
    closedLoopSetpoint = speeds;
  }

  public void updateTunables(){
    if (driveKp.hasChanged()
        || driveKd.hasChanged()
        || driveKs.hasChanged()
        || driveKv.hasChanged()
        || turnKp.hasChanged()
        || turnKd.hasChanged()) {
      driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
      for (int i = 0; i < 4; i++) {
        driveFeedback[i].setP(driveKp.get());
        driveFeedback[i].setD(driveKd.get());
        turnFeedback[i].setP(turnKp.get());
        turnFeedback[i].setD(turnKd.get());
      }
    }
  }
  private void updateOdometry(ModuleIOInputsAutoLogged[] moduleInputs){
    odometryPose=m_PoseEstimator.update(gyroInputs.rotation2d, new SwerveModulePosition[] {
      moduleInputs[0].pos,
      moduleInputs[1].pos,
      moduleInputs[2].pos,
      moduleInputs[3].pos
    });
    Optional<EstimatedRobotPose> result =
                pcw.getEstimatedGlobalPose(m_PoseEstimator.getEstimatedPosition());
    if(result.isPresent())
      m_PoseEstimator.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
    lastGyroPosRad = gyroInputs.positionRad;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void goToX() {
    driveMode = DriveMode.X;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxAngularSpeed;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return odometryPose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return odometryPose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometryPose = pose;
  }

  public Translation2d getFieldVelocity() {
    return fieldVelocity;
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)
    };
  }

  private static enum DriveMode {
    NORMAL,
    X
  }
}
