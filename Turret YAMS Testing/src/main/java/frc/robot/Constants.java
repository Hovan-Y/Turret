// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   public static enum AimPoints {
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
    RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
    BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

    public final Translation3d value;

    private AimPoints(Translation3d value) {
      this.value = value;
    }

    public static final Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TurretConstants {
    public static final int kMotorID = 4;

    public static final double kP = 50.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;

    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(90);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(45);
  }

  public static class HopperConstants {
    public static double speed = 0.0;
    public static int HopperMotorID = 1;
  }

  public static class IntakeConstants {
    public static int IntakeMotorID = 16;
    public static double INTAKE_SPEED = 1.0;
  }

  public static class FeederConstants {
    public static int FeederMotorID = 0;
  }

  public static class PivotConstants {
    public static int PivotMotorID = 0;
  }
}
