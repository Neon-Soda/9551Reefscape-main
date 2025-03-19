// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.OIConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Carriage.CarriageSystem;
import frc.robot.subsystems.Carriage.Elevator;
import frc.robot.subsystems.Carriage.Intake;
import frc.robot.subsystems.Carriage.Wrist;
import frc.robot.subsystems.Carriage.CarriageSystem.CarriageStates;
import frc.robot.subsystems.Carriage.Intake.IntakeStates;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller chassisController = new CommandPS5Controller(OIConstants.chassisControllerPort);
    private final CommandPS5Controller subsystemController = new CommandPS5Controller(OIConstants.subsystemControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Wrist wrist = new Wrist();
    private final CarriageSystem carriage = new CarriageSystem(elevator, intake, wrist);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-chassisController.getRightY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-chassisController.getRightX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-chassisController.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        NamedCommands.registerCommand("ReefL4", new InstantCommand(() -> {
            carriage.setState(CarriageStates.ReefL4);
        }));

        NamedCommands.registerCommand("CoralScore", new InstantCommand(() -> {
            intake.setCoralState(false);
            intake.setIfScoring(true);
            intake.setState(IntakeStates.Score);
        }));

        NamedCommands.registerCommand("StopCoralScore", new InstantCommand(() -> {
            intake.setState(IntakeStates.Stop);
            intake.setIfScoring(false);
            intake.setAlgaeState(false);
            intake.setAlgaeTransport(false);
        }));

        autoChooser = AutoBuilder.buildAutoChooser("Blue Auto");

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        

        chassisController.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        chassisController.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-chassisController.getLeftY(), -chassisController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        chassisController.create().and(chassisController.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        chassisController.create().and(chassisController.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        chassisController.options().and(chassisController.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        chassisController.options().and(chassisController.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        chassisController.L1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        subsystemController.L3().onTrue(new InstantCommand(() -> {
            intake.setState(IntakeStates.Intake, carriage.getState());
        })).toggleOnFalse(new InstantCommand(() -> {
            intake.setState(IntakeStates.Stop);
        }));

        subsystemController.touchpad().onTrue(new InstantCommand(() -> {
            intake.setCoralState(false);
            intake.setIfRegret(true);
            intake.setState(IntakeStates.Regret, carriage.getState());
        })).toggleOnFalse(new InstantCommand(() -> {
            intake.setCoralState(true);
            intake.setIfRegret(false);
            intake.setState(IntakeStates.Stop);
        }));

        subsystemController.R3().onTrue(new InstantCommand(() -> {
            intake.setCoralState(false);
            carriage.setState(CarriageStates.OffSet);
        }));

        subsystemController.cross().onTrue(new InstantCommand(() -> {
            intake.setCoralState(true);
            carriage.setState(CarriageStates.ReefL2);
        }));

        subsystemController.square().onTrue(new InstantCommand(() -> {
            intake.setCoralState(true);
            carriage.setState(CarriageStates.ReefL3);
        }));

        subsystemController.triangle().onTrue(new InstantCommand(() -> {
            intake.setCoralState(true);
            carriage.setState(CarriageStates.ReefL4);
        }));

        subsystemController.PS().onTrue(new InstantCommand(() -> {
            elevator.resetPosistion();
            wrist.setWristRotation(Carriage.wristOffSetPosition);
        }));

        subsystemController.povDown().onTrue(new InstantCommand(() -> {
            carriage.setState(CarriageStates.AlgaeL1);
        }));

        subsystemController.povUp().onTrue(new InstantCommand(() -> {
            carriage.setState(CarriageStates.AlgaeL2);
        }));

        subsystemController.povRight().onTrue(new InstantCommand(() -> {
            carriage.setState(CarriageStates.Processor);
        }));
        
        subsystemController.L1().onTrue(new InstantCommand(() -> {
            carriage.setState(CarriageStates.Net);
        })); 

        chassisController.R1().onTrue(new InstantCommand(() -> {
            intake.setCoralState(false);
            intake.setIfScoring(true);
            intake.setState(IntakeStates.Score);
        })).toggleOnFalse(new InstantCommand(() -> {
            intake.setState(IntakeStates.Stop);
            intake.setIfScoring(false);
            intake.setAlgaeState(false);
            intake.setAlgaeTransport(false);
        }));

        subsystemController.R1().onTrue(new InstantCommand(() -> {
            intake.setCoralState(false);
            intake.setIfScoring(true);
            intake.setState(IntakeStates.Score);
        })).toggleOnFalse(new InstantCommand(() -> {
            intake.setState(IntakeStates.Stop);
            intake.setIfScoring(false);
            intake.setAlgaeState(false);
            intake.setAlgaeTransport(false);
        }));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void resetRobotCentric() {
        drivetrain.seedFieldCentric();
    }
}
