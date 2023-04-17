// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroIONavX implements GyroIO {
  private final AHRS gyro;
  private final double[] xyzDps = new double[3];

  public GyroIONavX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        gyro = new AHRS(Port.kMXP);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIONavX");
    }
  }

  public void updateInputs(GyroIOInputs inputs) {
    // Again, the 2022 code base has good examples for reading this data. We generally prefer to use
    // "getAngle" instead of "getYaw" (what's the difference?)
    //
    // Remember to pay attention to the UNITS.
    xyzDps[0] = gyro.getRawGyroX();
    xyzDps[1] = gyro.getRawGyroY();
    xyzDps[2] = gyro.getRawGyroZ();
    inputs.connected = true;
    inputs.positionRad = Units.degreesToRadians(gyro.getYaw());
    inputs.velocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
    inputs.rotation2d = Rotation2d.fromDegrees(gyro.getYaw());
  }
}
