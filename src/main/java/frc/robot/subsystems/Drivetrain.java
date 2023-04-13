// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase implements AutoCloseable {
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.Drivetrain.TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.WHEELBASE_METERS / 2.0),
      new Translation2d(Constants.Drivetrain.TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.Drivetrain.TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.Drivetrain.TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.WHEELBASE_METERS / 2.0));

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.Drivetrain.PIGEON_ID);

  private double gyroOffset = 0;
  private final SwerveDriveOdometry m_odemetry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private final Field2d m_field = new Field2d();

  private final PIDController limelightPIDController = new PIDController(0.09, 0.02, 0.01);

  public Drivetrain() {
    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        Mk4iSwerveModuleHelper.GearRatio.L2,
        Constants.Drivetrain.FRONT_LEFT.DRIVE_MOTOR_ID,
        Constants.Drivetrain.FRONT_LEFT.STEER_MOTOR_ID,
        Constants.Drivetrain.FRONT_LEFT.ENCODER_ID,
        Constants.Drivetrain.FRONT_LEFT.STEER_OFFSET);

    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        Mk4iSwerveModuleHelper.GearRatio.L2,
        Constants.Drivetrain.FRONT_RIGHT.DRIVE_MOTOR_ID,
        Constants.Drivetrain.FRONT_RIGHT.STEER_MOTOR_ID,
        Constants.Drivetrain.FRONT_RIGHT.ENCODER_ID,
        Constants.Drivetrain.FRONT_RIGHT.STEER_OFFSET);

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        Mk4iSwerveModuleHelper.GearRatio.L2,
        Constants.Drivetrain.BACK_LEFT.DRIVE_MOTOR_ID,
        Constants.Drivetrain.BACK_LEFT.STEER_MOTOR_ID,
        Constants.Drivetrain.BACK_LEFT.ENCODER_ID,
        Constants.Drivetrain.BACK_LEFT.STEER_OFFSET);

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        Mk4iSwerveModuleHelper.GearRatio.L2,
        Constants.Drivetrain.BACK_RIGHT.DRIVE_MOTOR_ID,
        Constants.Drivetrain.BACK_RIGHT.STEER_MOTOR_ID,
        Constants.Drivetrain.BACK_RIGHT.ENCODER_ID,
        Constants.Drivetrain.BACK_RIGHT.STEER_OFFSET);

    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Fully resets the gyroscope angle. You probably want calibrateGyroscope()
   * instead.
   */
  public void zeroGyroscope() {
    m_pigeon.setYaw(0.0);
  }

  /**
   *
   */
  public void calibrateGyroscope() {
    gyroOffset = -m_pigeon.getYaw();
  }

  public void setGyroscope(double angle) {
    m_pigeon.setYaw(angle);
  }

  public Rotation2d getRawGyroscope() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw() + gyroOffset);
  }

  public SwerveModuleState getState(SwerveModule module) {
    return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
  }

  public void setState(SwerveModule module, SwerveModuleState state) {
    module.set(state.speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS * Constants.Swerve.MAX_VOLTAGE,
        state.angle.getRadians());
  }

  public void setAllStates(SwerveModuleState[] states) {
    setState(m_frontLeftModule, states[0]);
    setState(m_frontRightModule, states[1]);
    setState(m_backLeftModule, states[2]);
    setState(m_backRightModule, states[3]);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_VELOCITY_METERS);
    setAllStates(states);
  }

  public Pose2d getPose2d() {
    return m_odemetry.getPoseMeters();
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getState(m_frontLeftModule), getState(m_frontRightModule),
        getState(m_backRightModule), getState(m_backLeftModule));
  }

  public void resetOdometry(Pose2d resetPos) {
    m_odemetry.resetPosition(resetPos, resetPos.getRotation());
  }

  public void stopModules() {
    m_frontLeftModule.set(0, 0);
    m_frontRightModule.set(0, 0);
    m_backLeftModule.set(0, 0);
    m_backRightModule.set(0, 0);
  }

  public void defense() {
    m_frontLeftModule.set(0, -45);
    m_frontRightModule.set(0, 45);
    m_backLeftModule.set(0, 45);
    m_backRightModule.set(0, -45);
  }

  public PIDController getLimelightPID() {
    return limelightPIDController;
  }

  @Override
  public void periodic() {
    m_odemetry.update(getRawGyroscope(), getState(m_frontLeftModule), getState(m_frontRightModule),
        getState(m_backLeftModule), getState(m_backRightModule));
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("Drivetrain/Gyro", getRawGyroscope().getDegrees());
    m_field.setRobotPose(getPose2d());
    SmartDashboard.putNumber("Drivetrain/WheelAngle/FrontLeft", getState(m_frontLeftModule).angle.getDegrees());
    SmartDashboard.putNumber("Drivetrain/WheelAngle/FrontRight", getState(m_frontRightModule).angle.getDegrees());
    SmartDashboard.putNumber("Drivetrain/WheelAngle/BackLeft", getState(m_backLeftModule).angle.getDegrees());
    SmartDashboard.putNumber("Drivetrain/WheelAngle/BackRight", getState(m_backRightModule).angle.getDegrees()); 
  }

  @Override
  public void simulationPeriodic() {
    m_field.setRobotPose(m_odemetry.getPoseMeters());
  }

  @Override
  public void close() throws Exception {

  }
}
