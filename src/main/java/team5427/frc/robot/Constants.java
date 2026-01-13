// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5427.frc.robot;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team5427.lib.motors.MotorConfiguration;
import team5427.lib.motors.MotorConfiguration.IdleState;
import team5427.lib.motors.MotorConfiguration.MotorMode;
import team5427.lib.motors.MotorUtil;
import team5427.lib.systems.swerve.SwerveUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kCanivoreBusName = "canivore_bus_name";
  public static final double kOdometryFrequency =
      new CANBus(Constants.kCanivoreBusName).isNetworkFD() ? 250.0 : 100.0;

  public static final Frequency kHighPriorityUpdateFrequency = Hertz.of(100.0);
  public static final Frequency kMediumPriorityUpdateFrequency = Hertz.of(50.0);
  public static final Frequency kLowPriorityUpdateFrequency = Hertz.of(10.0);

  public static Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ModeTriggers {
    public static final Trigger kReal =
        new Trigger(
            () -> {
              return currentMode.equals(Mode.REAL);
            });
    public static final Trigger kSim =
        new Trigger(
            () -> {
              return currentMode.equals(Mode.SIM);
            });
    public static final Trigger kReplay =
        new Trigger(
            () -> {
              return currentMode.equals(Mode.REPLAY);
            });
  }

  public static final double kLoopSpeed = Units.millisecondsToSeconds(20);

  public static final boolean kIsTuningMode = true;

  public static RobotConfig config;

  public static class DriverConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kOperatorJoystickPort = 1;
    public static final double kDriverControllerJoystickDeadzone = 0.0;
    public static final double kDriverControllerRotationalControlJoystickDeadzone = 0.05;
  }

  public static class SwerveConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(2.195 * 2.0);
    public static final double kTrackWidth = Units.inchesToMeters(22.75);
    public static final double kWheelBase = Units.inchesToMeters(22.75);

    public static MotorConfiguration kDriveMotorConfiguration = new MotorConfiguration();

    static {
      kDriveMotorConfiguration.gearRatio = SwerveUtil.kSDSL3GearRatio;
      kDriveMotorConfiguration.idleState = IdleState.kBrake;
      kDriveMotorConfiguration.mode = MotorMode.kFlywheel;
      kDriveMotorConfiguration.withFOC = true;

      kDriveMotorConfiguration.currentLimit = 80;
      kDriveMotorConfiguration.finalDiameterMeters = kWheelDiameterMeters;

      kDriveMotorConfiguration.maxVelocity =
          kDriveMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM);
      kDriveMotorConfiguration.maxAcceleration = kDriveMotorConfiguration.maxVelocity * 2.0;

      kDriveMotorConfiguration.kP = 70.789; // 2.64 , 30.64
      // kDriveMotorConfiguration.kV = 0.75;
      kDriveMotorConfiguration.kA = 6.0;
      kDriveMotorConfiguration.kS = 0.5;
      kDriveMotorConfiguration.altV = kDriveMotorConfiguration.maxVelocity;
      kDriveMotorConfiguration.altA = kDriveMotorConfiguration.maxAcceleration;

    }

    public static final double kAutoAlignRotationalKp = 0.2; // 0.09
    public static final double kAutoAlignRotationalKd = 0.02; // 0.02
    public static final double kAutoAlignTranslationKp = 2.0; // 5.0 , 2.7
    public static final double kAutoAlignTranslationKd = 0.2; // 0.9
    public static final double kAutoAlignServoTranslationalKp = 1.0;
  }
}
