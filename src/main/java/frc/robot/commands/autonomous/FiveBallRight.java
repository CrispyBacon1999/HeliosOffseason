// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.LimelightShoot;
import frc.robot.commands.MagazineAutoBump;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class FiveBallRight extends AutoBaseCommand {
        PPSwerveControllerCommand first3Cargo;
        PPSwerveControllerCommand last2Cargo;

        public FiveBallRight(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
                        Limelight limelight) {
                super(drivetrain, shooter, intake, magazine, climber, limelight);

                addCommands(
                                new InstantCommand(() -> {
                                        drivetrain.setGyroscope(90);
                                        drivetrain.resetOdometry(new Pose2d(7.59, 1.75, Rotation2d.fromDegrees(90)));
                                }),
                                new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                                first3Cargo,
                                                                new PrintCommand("Made it to location A"),
                                                                new InstantCommand(() -> drivetrain.stopModules()),
                                                                new PrintCommand("Aiming!"),
                                                                new LimelightAim(drivetrain, limelight)
                                                                                .withTimeout(1.5),
                                                                new PrintCommand("Aimed!")),
                                                new IntakeCargo(intake, magazine)
                                                                .alongWith(new MagazineAutoBump(magazine))),
                                new SequentialCommandGroup(
                                                new PrintCommand("Shooting 1!"),
                                                new LimelightShoot(shooter, magazine, limelight),
                                                new IntakeCargo(intake, magazine)
                                                                .alongWith(new MagazineAutoBump(magazine))
                                                                .withTimeout(1.5),
                                                new PrintCommand("Shooting 2!"),
                                                new LimelightShoot(shooter, magazine, limelight).withTimeout(1.5),
                                                new PrintCommand("Shooting 3!"),
                                                new LimelightShoot(shooter, magazine, limelight).withTimeout(1.5)),
                                new SequentialCommandGroup(
                                                last2Cargo.alongWith(new IntakeCargo(intake, magazine)
                                                                .alongWith(new MagazineAutoBump(magazine))),
                                                new LimelightAim(drivetrain, limelight).withTimeout(1.5),
                                                new PrintCommand("Shooting 1!"),
                                                new LimelightShoot(shooter, magazine, limelight).withTimeout(2),
                                                new PrintCommand("Shooting 2!"),
                                                new LimelightShoot(shooter, magazine, limelight).withTimeout(2)));
        }

        protected void generatePaths() {
                PathPlannerTrajectory m_first3Cargo = PathPlanner.loadPath("Five Ball A", 6, 4);
                PathPlannerTrajectory m_last2Cargo = PathPlanner.loadPath("Five Ball B", 6, 2);

                first3Cargo = new PPSwerveControllerCommand(
                                m_first3Cargo,
                                drivetrain::getPose2d,
                                drivetrain.getKinematics(),
                                m_translationController,
                                m_strafeController,
                                m_thetaController,
                                drivetrain::setAllStates,
                                drivetrain);

                last2Cargo = new PPSwerveControllerCommand(
                                m_last2Cargo, drivetrain::getPose2d,
                                drivetrain.getKinematics(),
                                m_translationController,
                                m_strafeController,
                                m_thetaController,
                                drivetrain::setAllStates,
                                drivetrain);

        }
}
