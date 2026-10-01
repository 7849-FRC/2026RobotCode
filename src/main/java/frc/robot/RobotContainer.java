// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.Constants.DriveTrain.AIM_D;
import static frc.robot.Constants.DriveTrain.AIM_DEADBAND;
import static frc.robot.Constants.DriveTrain.AIM_I;
import static frc.robot.Constants.DriveTrain.AIM_P;
import static frc.robot.Constants.DriveTrain.MAX_ANGULAR_RATE;
import static frc.robot.Constants.DriveTrain.MAX_SPEED;
import static frc.robot.Constants.Operator.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.Operator.MAJOR_CREEP_NERF_DRIVE;
import static frc.robot.Constants.Operator.MAJOR_CREEP_NERF_ROTATE;
import static frc.robot.Constants.Operator.OPERATOR_CONTROLLER_PORT;
import static frc.robot.Constants.Operator.SLIGHT_CREEP_NERF_DRIVE;
import static frc.robot.Constants.Operator.SLIGHT_CREEP_NERF_ROTATE;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.cmds.AimTimed;
import frc.robot.cmds.BounceIntake;
import frc.robot.cmds.IntakeTimed;
import frc.robot.cmds.ShootAtCalculatedVelocity;
import frc.robot.cmds.ShootAtSetSpeed;
import frc.robot.cmds.ShootAtSetSpeedTimed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import lib.RobotMethods;

public final class RobotContainer implements RobotMethods {
        private final CommandSwerveDrivetrain drivetrain;
        private final Indexer indexer;
        private final Intake intake;
        // private final Vision vision;
        private final Shooter shooter;

        private final CommandXboxController joystick;
        private final CommandXboxController operator;

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive; // Use open-loop control for drive
                                                        // motors

        private final SwerveRequest.FieldCentricFacingAngle aiming;

        private final SendableChooser<Command> autoChooser;

        private final Timer timer = new Timer();

        private Rotation2d latestAngleToHubTargetCenter;

        public RobotContainer() {
                drivetrain = TunerConstants.createDrivetrain();
                indexer = Indexer.getInstance();
                intake = Intake.getInstance();
                // vision = Vision.getInstance();
                shooter = Shooter.getInstance();

                drivetrain.initialize();
                indexer.initialize();
                intake.initialize();
                // vision.initialize();
                shooter.initialize();

                joystick = new CommandXboxController(DRIVER_CONTROLLER_PORT);
                operator = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

                /* Setting up bindings for necessary control of the swerve drive platform */
                drive = new SwerveRequest.FieldCentric()
                                .withDeadband(MAX_SPEED * 0.1).withRotationalDeadband(MAX_ANGULAR_RATE * 0.1) // Add a
                                                                                                              // 10%
                                                                                                              // deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for
                                                                                         // drive
                                                                                         // motors

                aiming = new FieldCentricFacingAngle()
                                .withHeadingPID(AIM_P, AIM_I, AIM_D)
                                .withRotationalDeadband(AIM_DEADBAND);

                configureDefault();
                configureBindings();
                registerNamedCommands();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto", autoChooser);

                // clean up garbage after initialization
                timer.start();

                latestAngleToHubTargetCenter = Rotation2d.fromDegrees(0);
        }

        private void configureDefault() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MAX_SPEED) // Drive
                                                                                                                    // forward
                                                                                                                    // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MAX_SPEED) // Drive left with
                                                                                                 // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MAX_ANGULAR_RATE) // Drive
                                                                                                              // counterclockwise
                                                                                                              // with
                                // negative X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        }

        private void configureBindings() {
                // ── Driver Controls ────────────────────────────────────────────────────────

                // Reset field-centric heading
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Slight creep mode
                joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() -> drive
                                .withVelocityX(-joystick.getLeftY() * MAX_SPEED * SLIGHT_CREEP_NERF_DRIVE)
                                .withVelocityY(-joystick.getLeftX() * MAX_SPEED * SLIGHT_CREEP_NERF_DRIVE)
                                .withRotationalRate(
                                                -joystick.getRightX() * MAX_ANGULAR_RATE * SLIGHT_CREEP_NERF_ROTATE)));

                // Major creep mode
                joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() -> drive
                                .withVelocityX(-joystick.getLeftY() * MAX_SPEED * MAJOR_CREEP_NERF_DRIVE)
                                .withVelocityY(-joystick.getLeftX() * MAX_SPEED * MAJOR_CREEP_NERF_DRIVE)
                                .withRotationalRate(
                                                -joystick.getRightX() * MAX_ANGULAR_RATE * MAJOR_CREEP_NERF_ROTATE)));

                operator.rightBumper().whileTrue(
                                new ParallelCommandGroup(
                                                Commands.run(
                                                                intake.runIntakeMotor()),
                                                Commands.run(
                                                                intake.feedNewFeeder()),
                                                Commands.run(
                                                                indexer.runFeeder(44))))
                                .onFalse(
                                                new ParallelCommandGroup(
                                                                Commands.run(
                                                                                intake.stopIntakeMotor()),
                                                                Commands.run(
                                                                                intake.stopNewFeeder()),
                                                                Commands.run(indexer.stopFeeder())));
                operator.leftBumper().whileTrue(
                                new ParallelCommandGroup(
                                                Commands.run(
                                                                intake.runIntakeMotor()),
                                                Commands.run(
                                                                intake.runNewFeeder())))
                                .onFalse(
                                                new ParallelCommandGroup(
                                                                Commands.run(
                                                                                intake.stopIntakeMotor()),
                                                                Commands.run(
                                                                                intake.stopNewFeeder())));

                // Aim drivetrain at hub target
                // joystick.a()
                // .whileTrue(drivetrain.applyRequest(() -> aiming
                // .withVelocityX(Math.abs(joystick.getLeftY()) > 0.1
                // ? -joystick.getLeftY()
                // * Constants.DriveTrain.AIM_MOVEMENT_NERF
                // : 0)
                // .withVelocityY(Math.abs(joystick.getLeftX()) > 0.1
                // ? -joystick.getLeftX()
                // * Constants.DriveTrain.AIM_MOVEMENT_NERF
                // : 0)
                // .withTargetDirection(latestAngleToHubTargetCenter)))
                // .onFalse(drivetrain.applyRequest(() -> drive
                // .withVelocityX(-joystick.getLeftY() * MAX_SPEED)
                // .withVelocityY(-joystick.getLeftX() * MAX_SPEED)
                // .withRotationalRate(-joystick.getRightX() * MAX_ANGULAR_RATE)));

                // // ── Operator Controls
                // ──────────────────────────────────────────────────────

                // // Shoot at calculated velocity
                // operator.a()
                // .whileTrue(
                // !intake.getState().equals(intake.getOutType())
                // ? new ParallelCommandGroup(
                // new ShootAtCalculatedVelocity(shooter,
                // indexer, vision),
                // new BounceIntake(intake))

                // :

                // new SequentialCommandGroup(
                // new PivotIntake(intake),
                // new ParallelCommandGroup(
                // new ShootAtCalculatedVelocity(
                // shooter,
                // indexer,
                // vision))))
                // .onFalse(
                // new ParallelCommandGroup(
                // Commands.runOnce(shooter.stop()),
                // Commands.runOnce(indexer.stage1Off()),
                // Commands.runOnce(indexer.stopFeeder()),
                // Commands.runOnce(intake.stopPivot())));

                // Shooter velocity presets
                // operator.leftBumper()
                // .whileTrue(
                // !intake.getState().equals(intake.getOutType())
                // ? new ParallelCommandGroup(
                // new ShootAtSetSpeed(shooter, indexer,
                // Constants.Shooter.SHOOTER_MAX_RPS))

                // :

                // new SequentialCommandGroup(
                // new PivotIntake(intake),
                // new ParallelCommandGroup(
                // new ShootAtSetSpeed(
                // shooter,
                // indexer,
                // Constants.Shooter.SHOOTER_MAX_RPS))))
                // .onFalse(
                // new ParallelCommandGroup(
                // Commands.runOnce(shooter.stop()),
                // Commands.runOnce(indexer.stage1Off()),
                // Commands.runOnce(indexer.stopFeeder()),
                // Commands.runOnce(intake.stopPivot())));

                // operator.rightBumper()
                // .whileTrue(
                // !intake.getState().equals(intake.getOutType())
                // ? new ParallelCommandGroup(
                // new ShootAtSetSpeed(shooter, indexer,
                // Constants.Shooter.HALF_FIELD_RPS))

                // :

                // new SequentialCommandGroup(
                // new PivotIntake(intake),
                // new ParallelCommandGroup(
                // new ShootAtSetSpeed(
                // shooter,
                // indexer,
                // Constants.Shooter.HALF_FIELD_RPS))))
                // .onFalse(
                // new ParallelCommandGroup(
                // Commands.runOnce(shooter.stop()),
                // Commands.runOnce(indexer.stage1Off()),
                // Commands.runOnce(indexer.stopFeeder()),
                // Commands.runOnce(intake.stopPivot())));

                operator.leftStick()
                                .whileTrue(Commands.run(shooter.setVelocity(-10)))
                                .onFalse(Commands.run(shooter.stop()));

                // Indexer stage 1 (POV left & up both trigger oscillation)
                operator.povLeft()
                                .whileTrue(Commands.run(indexer.stage1On()))
                                .onFalse(Commands.run(indexer.stage1Off()));

                // operator.povDown()
                // .whileTrue(Commands.run(intake.outake()))
                // .onFalse(Commands.run(intake.stopIntake()));

                // // Intake + indexer together
                // operator.b()
                // .whileTrue(Commands.run(intake.intake()))
                // .onFalse(Commands.run(intake.stopIntake()));

                // // Intake pivot manual control
                // operator.back()
                // .whileTrue(Commands.run(intake.runPivotRawIn()))
                // .onFalse(Commands.run(intake.stopPivot()));

                // operator.rightStick()
                // .whileTrue(Commands.run(intake.runPivotRawOut()))
                // .onFalse(Commands.run(intake.stopPivot()));

                // operator.povUp().onTrue(Commands.runOnce(intake.zeroPivot()));

                // operator.povRight().whileTrue(Commands.run(indexer.stage1Back()))
                // .onFalse(Commands.run(indexer.stopFeeder()));

                // operator.x().onTrue(new PivotIntake(intake));

                // Shooter velocity presets
                // operator.leftBumper()
                // .whileTrue(
                // !intake.getState().equals(intake.getOutType())
                // ? new ParallelCommandGroup(
                // new ShootAtSetSpeed(shooter, indexer,
                // Constants.Shooter.SHOOTER_MAX_RPS))

                // :

                // new SequentialCommandGroup(
                // new PivotIntake(intake),
                // new ParallelCommandGroup(
                // new ShootAtSetSpeed(
                // shooter,
                // indexer,
                // Constants.Shooter.SHOOTER_MAX_RPS))))
                // .onFalse(
                // new ParallelCommandGroup(
                // Commands.runOnce(shooter.stop()),
                // Commands.runOnce(indexer.stage1Off()),
                // Commands.runOnce(indexer.stopFeeder()),
                // Commands.runOnce(intake.stopPivot())));

                // operator.rightTrigger()
                // .whileTrue(
                // !intake.getState().equals(intake.getOutType())
                // ? new ParallelCommandGroup(
                // new ShootAtSetSpeed(shooter, indexer,
                // 65))

                // :

                // new SequentialCommandGroup(
                // new PivotIntake(intake),
                // new ParallelCommandGroup(
                // new ShootAtSetSpeed(
                // shooter,
                // indexer,
                // Constants.Shooter.HALF_FIELD_RPS))))
                // .onFalse(
                // new ParallelCommandGroup(
                // Commands.runOnce(shooter.stop()),
                // Commands.runOnce(indexer.stage1Off()),
                // Commands.runOnce(indexer.stopFeeder()),
                // Commands.runOnce(intake.stopPivot())));

                operator.leftTrigger()
                                .whileTrue(
                                                Commands.run(
                                                                shooter.runFullSpeedRaw()))
                                .onFalse(
                                                Commands.run(
                                                                shooter.stop()));

                // :

                // new SequentialCommandGroup(
                // new PivotIntake(intake),
                // new ParallelCommandGroup(
                // new ShootAtSetSpeed(
                // shooter,
                // indexer,
                // Constants.Shooter.HALF_FIELD_RPS))))
                // .onFalse(
                // new ParallelCommandGroup(
                // Commands.runOnce(shooter.stop()),
                // Commands.runOnce(indexer.stage1Off()),
                // Commands.runOnce(indexer.stopFeeder()),
                // Commands.runOnce(intake.stopPivot())));

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void registerNamedCommands() {
                NamedCommands.registerCommand("midshoot",
                                new ShootAtSetSpeedTimed(shooter, indexer, 55, 5, shooter, indexer));
                // NamedCommands.registerCommand("aim", new AimTimed(drivetrain, vision, aiming,
                // 1, drivetrain, vision));
                // NamedCommands.registerCommand("pivotintake", new PivotIntake(intake));
                // NamedCommands.registerCommand("Intake", new IntakeTimed(intake, 5, intake));
        }

        @Override
        public void robotPeriodic() {
                // periodically call garbage collector
                if (timer.advanceIfElapsed(5)) {
                        System.gc();
                }

                // always be tracking the latest angle we need to aim the robot at
                // latestAngleToHubTargetCenter = vision
                // .calculateRobotOffsetToTargetCenter(drivetrain.getPose().getRotation());
        }

        @Override
        public void disabledInit() {

        }

        @Override
        public void disabledPeriodic() {

        }

        @Override
        public void disabledExit() {

        }

        @Override
        public void autonomousInit() {

        }

        @Override
        public void autonomousPeriodic() {

        }

        @Override
        public void autonomousExit() {
                drivetrain.setControl(drive.withVelocityX(-joystick.getLeftY() * MAX_SPEED)
                                .withVelocityY(-joystick.getLeftX() * MAX_SPEED)
                                .withRotationalRate(-joystick.getRightX() * MAX_ANGULAR_RATE));

                shooter.stop().run();
                // intake.stopIntake().run();
                // intake.stopPivot().run();
                indexer.stopFeeder().run();
                indexer.stage1Off().run();
        }

        @Override
        public void teleopInit() {
                drivetrain.setControl(drive.withVelocityX(-joystick.getLeftY() * MAX_SPEED)
                                .withVelocityY(-joystick.getLeftX() * MAX_SPEED)
                                .withRotationalRate(-joystick.getRightX() * MAX_ANGULAR_RATE));

                shooter.stop().run();
                // intake.stopIntake().run();
                // intake.stopPivot().run();
                indexer.stopFeeder().run();
                indexer.stage1Off().run();
        }

        @Override
        public void teleopPeriodic() {

        }

        @Override
        public void teleopExit() {

        }

        @Override
        public void testInit() {

        }

        @Override
        public void testPeriodic() {

        }

        @Override
        public void testExit() {

        }

        @Override
        public void simulationPeriodic() {

        }
}
