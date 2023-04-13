// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import static java.util.Map.entry;
import java.util.TreeMap;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve {
        public static final double TELEOP_SPEED = 1;
        public static final double MAX_VELOCITY_METERS = 6380.0 / 60.0
                * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

        public static final double MAX_VOLTAGE = 12;

        public static final double MAX_ANG_ACCEL = MAX_VELOCITY_METERS
                / Math.hypot(Constants.Drivetrain.TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.WHEELBASE_METERS / 2.0);
    }
//Intake is Front
    public static final class Drivetrain {
        public static final int PIGEON_ID = 13;
        public static final double WHEELBASE_METERS = 0.749;
        public static final double TRACKWIDTH_METERS = 0.749;
        public static final SwerveConstants FRONT_LEFT = new SwerveConstants(1, 2, 9, 189.31171875);
        public static final SwerveConstants FRONT_RIGHT = new SwerveConstants(3, 4, 10, 343.473046875);
        public static final SwerveConstants BACK_LEFT = new SwerveConstants(5, 6, 11, 88.93828125-180);
        public static final SwerveConstants BACK_RIGHT = new SwerveConstants(7, 8, 12, 45.084375-180);
    }

    public static final class Magazine {
        public static final int LOWER_MOTOR = 16;
        public static final int UPPER_MOTOR = 17;
        public static final int LOWER_SENSOR = 1;
        public static final int UPPER_SENSOR = 0;

        public static final double UPPER_SENSOR_THRESHOLD = 1000;
        public static final double LOWER_SENSOR_THRESHOLD = 1000;
    }

    public static final class Intake {
        public static final int MOTOR_ID = 18;
        public static final int SOLENOID_ID = 0;
    }

    public static final class Shooter {
        public static final int LEFT_MOTOR_ID = 19;
        public static final int RIGHT_MOTOR_ID = 20;
        public static final int HOOD_MOTOR_ID = 21;

        public static final double kP = .3;
        public static final double kI = 0.01;
        public static final double kD = 0.01;
        public static final double kF = 0.0639;
    }

    public static final class Turret {
        public static final int MOTOR_ID = 22;

        public static final double kP = .25;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final int ENCODER_COUNTS_PER_REV = 42;
        public static final int GEAR_RATIO = 4; // TODO: CHANGE THIS TO ACTUAL GEAR RATIO

        public static final double CLIMBER_POLE_DEADZONE_CENTER = 45;
        public static final double CLIMBER_POLE_DEADZONE_WIDTH = 8;

        // Maximum front angle the turret can turn to with climber arms up.
        // Not used yet, will be using turret only if arms are down currently.
        public static final double CLIMBER_ARM_LEFT_ANGLE = 2;
        public static final double CLIMBER_ARM_RIGHT_ANGLE = 2;

        public static final TreeMap<Double, Double> TIME_OF_FLIGHT = new TreeMap<>(
                Map.ofEntries(
                        entry(2.5, 1.0),
                        entry(3.0, 1.05),
                        entry(3.5, 1.10),
                        entry(4.0, 1.15),
                        entry(4.5, 1.32),
                        entry(5.0, 1.4),
                        entry(5.5, 1.52)));
    }

    public static final class Climber {
        public static final int LEFT_MOTOR_ID = 14;
        public static final int RIGHT_MOTOR_ID = 15;
        public static final int SOLENOID_ID = 1;

        public static final double kP = 0.0001;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class Auto {
        private static final double MAX_ANG_VEL_RAD_AUTO = .4 * Math.PI; // .25

        public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(
                MAX_ANG_VEL_RAD_AUTO, Constants.Swerve.MAX_ANG_ACCEL);

        public static final PIDController X_PID_CONTROLLER = new PIDController(5, 0, 0); // 5
        // y distance PID controller
        public static final PIDController Y_PID_CONTROLLER = new PIDController(5, 0, 0); // 5, 0, .0 0.3, 0.4, 4
        // ROTATION (angle) PID controller
        public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(-3, 0, 0, // .85 works
                ROT_PROFILE);
        public static final TrajectoryConfig T_CONFIG = new TrajectoryConfig(Constants.Swerve.MAX_VELOCITY_METERS, 4);
    }

    public static final class Field {

        public static final Translation2d HUB_LOCATION = new Translation2d(Units.inchesToMeters(324),
                Units.inchesToMeters(162));

        public static final Translation2d FIELD_SIZE = new Translation2d(Units.inchesToMeters(648),
                Units.inchesToMeters(324));

        public static final double LOW_RUNG_HEIGHT = Units.inchesToMeters(48.75);
        public static final double MID_RUNG_HEIGHT = Units.inchesToMeters(60.25);
    }

}
