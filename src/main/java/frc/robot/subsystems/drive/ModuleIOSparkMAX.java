// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.SparkMaxDerivedVelocityController;

public class ModuleIOSparkMAX implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final SparkMaxDerivedVelocityController driveDerivedVelocityController;
  private final RelativeEncoder driveDefaultEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANCoder turnAbsoluteEncoder;

  private final double driveAfterEncoderReduction = 8.14;
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final boolean isAbsoluteEncoderInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMAX(int index) {
    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        switch (index) {
          case 0:
            driveSparkMax = new CANSparkMax(10, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(23);
            absoluteEncoderOffset = Rotation2d.fromDegrees(34.5);
            break;
          case 1:
            driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(21);
            absoluteEncoderOffset = Rotation2d.fromDegrees(192.65);
            break;
          case 2:
            driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(22);
            absoluteEncoderOffset = Rotation2d.fromDegrees(254.4);
            break;
          case 3:
            driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
            turnAbsoluteEncoder = new CANCoder(24);
            absoluteEncoderOffset = Rotation2d.fromDegrees(159.2);
            break;
          default:
            throw new RuntimeException("Invalid module index for ModuleIOSparkMAX");
        }
        break;
      default:
        throw new RuntimeException("Invalid robot for ModuleIOSparkMAX");
    }

    turnSparkMax.setInverted(isTurnMotorInverted);

    driveSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveDerivedVelocityController = new SparkMaxDerivedVelocityController(driveSparkMax);
    driveDefaultEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnRelativeEncoder.setPosition(0.0);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveDerivedVelocityController.getPosition())
            / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveDerivedVelocityController.getVelocity())
            / driveAfterEncoderReduction;
    inputs.driveVelocityFilteredRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveDefaultEncoder.getVelocity())
            / driveAfterEncoderReduction;
    inputs.driveAppliedVolts =
        driveSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

    inputs.turnAbsolutePositionRad =
         Rotation2d.fromDegrees(turnAbsoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset)
            .getRadians();
    inputs.turnPositionRad =
        Units.rotationsToRadians(turnRelativeEncoder.getPosition()) / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / turnAfterEncoderReduction;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
    inputs.pos = new SwerveModulePosition(inputs.drivePositionRad*Units.inchesToMeters(2), Rotation2d.fromRadians(inputs.turnAbsolutePositionRad));
    inputs.state = new SwerveModuleState(inputs.driveVelocityRadPerSec*Units.inchesToMeters(2), Rotation2d.fromRadians(inputs.turnAbsolutePositionRad));

  }

  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
