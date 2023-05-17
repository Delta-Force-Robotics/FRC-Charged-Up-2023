package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.SwerveJoystickCommand;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.ElevatorConstants;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.CamMode;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.LEDState;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LimelightSubsystem limeLightSubsystem = new LimelightSubsystem("tx", "ty", "ta");
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick driver2Joystick = new Joystick(OIConstants.kDriver2ControllerPort);
    private boolean configured = false;

    private final SendableChooser autoStartSideChooser = new SendableChooser<String>();
    private final SendableChooser autoRoutineChooser = new SendableChooser<String>();

    public RobotContainer() {
        autoStartSideChooser.addOption("Blue", "BlueSide");
        autoStartSideChooser.addOption("Red", "RedSide");

        autoRoutineChooser.addOption("Mid", "AutoMid");
        autoRoutineChooser.addOption("Cable Cover", "AutoCableCover");
        autoRoutineChooser.addOption("Open", "AutoOpen");

        SmartDashboard.putData(autoStartSideChooser);
        SmartDashboard.putData(autoRoutineChooser);
    }

    public void teleOpInit() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> DriveConstants.kFieldCentric));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        configured = true;

        new POVButton(driverJoystick, 270).onTrue(Commands.runOnce(() -> {
            if (limeLightSubsystem.currCamMode == CamMode.VISION_PROCESSOR) {
                limeLightSubsystem.setCamMode(CamMode.DRIVER_CAMERA);
            } else {
                limeLightSubsystem.setCamMode(CamMode.VISION_PROCESSOR);
            }
        }, limeLightSubsystem));

        new POVButton(driverJoystick, 90).onTrue(Commands.runOnce(() -> {
            if (limeLightSubsystem.currLedState == LEDState.PIPELINE) {
                limeLightSubsystem.setLedMode(LEDState.BLINK);
            } else {
                limeLightSubsystem.setLedMode(LEDState.PIPELINE);
            }
        }, limeLightSubsystem));
    }

    public Command getAutonomousCommand() {
        if(autoStartSideChooser.getSelected() == "BlueSide") {
            if(autoRoutineChooser.getSelected() == "AutoOpen") {
                return new SequentialCommandGroup(new SequentialCommandGroup(Commands.runOnce(() -> {
                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCone);

                    double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }
                    
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OUTTAKE);

                    zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }

                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                    intakeSubsystem.setSetpoint(IntakeConstants.kAngleRadHome);
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                }, elevatorSubsystem, intakeSubsystem)
                ));
            }
            else if(autoRoutineChooser.getSelected() == "AutoMid") {
                return new SequentialCommandGroup(new SequentialCommandGroup(Commands.runOnce(() -> {
                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                    double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }
                    
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OUTTAKE);

                    zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }

                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                    intakeSubsystem.setSetpoint(IntakeConstants.kAngleRadHome);
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                }, elevatorSubsystem, intakeSubsystem)
                ));
            }
            else {
                return new SequentialCommandGroup(new SequentialCommandGroup(Commands.runOnce(() -> {
                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCone);

                    double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }
                    
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OUTTAKE);

                    zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }

                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                    intakeSubsystem.setSetpoint(IntakeConstants.kAngleRadHome);
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                }, elevatorSubsystem, intakeSubsystem)
                ));
            }
        }
        else {
            if(autoRoutineChooser.getSelected() == "AutoOpen") {
                return new SequentialCommandGroup(new SequentialCommandGroup(Commands.runOnce(() -> {
                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCone);

                    double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }
                    
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OUTTAKE);

                    zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }

                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                    intakeSubsystem.setSetpoint(IntakeConstants.kAngleRadHome);
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                }, elevatorSubsystem, intakeSubsystem)
                ));
            }
            else if(autoRoutineChooser.getSelected() == "AutoMid") {
                return new SequentialCommandGroup(new SequentialCommandGroup(Commands.runOnce(() -> {
                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                    double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }
                    
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OUTTAKE);

                    zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }

                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                    intakeSubsystem.setSetpoint(IntakeConstants.kAngleRadHome);
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                }, elevatorSubsystem, intakeSubsystem)
                ));
            }
            else {
                return new SequentialCommandGroup(new SequentialCommandGroup(Commands.runOnce(() -> {
                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCone);

                    double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }
                    
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OUTTAKE);

                    zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    elevatorSubsystem.periodic();
                                    intakeSubsystem.periodic();
                                }

                    elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                    intakeSubsystem.setSetpoint(IntakeConstants.kAngleRadHome);
                    intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                }, elevatorSubsystem, intakeSubsystem)
                ));
            }
        }
    }

    //public Command getAutonomousCommand() {}
            /*clawSubsystem.currWheelDirection = WheelDirection.INTAKE;
            clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
            ParkCommand parkCommand = new ParkCommand(swerveSubsystem);

            if(autoStartSideChooser.getSelected() == "BlueSide") {
                if(autoRoutineChooser.getSelected() == "AutoOpen") {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("OpenExitComunityPathBlue", new PathConstraints(4, 4));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else if (autoRoutineChooser.getSelected() == "AutoMid"){
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("MiddleExitComunityPathBlue", new PathConstraints(2, 2));
                    PathPlannerTrajectory trajStrafe = PathPlanner.loadPath("MiddleStrafePathBlue", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);
                    Command traj2 = swerveSubsystem.followTrajectoryCommand(trajStrafe, false);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        parkCommand,
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("CableCoverExitComunityPathBlue", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
            }
            else {
                if(autoRoutineChooser.getSelected() == "AutoOpen") {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("OpenExitComunityPathRed", new PathConstraints(4, 4));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else if(autoRoutineChooser.getSelected() == "AutoMid") {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("MiddleExitComunityPathRed", new PathConstraints(2, 2));
                    PathPlannerTrajectory trajStrafe = PathPlanner.loadPath("MiddleStrafePathRed", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);
                    Command traj2 = swerveSubsystem.followTrajectoryCommand(trajStrafe, false);


                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    clawSubsystem.periodic();
                                    armSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        parkCommand,
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("CableCoverExitComunityRedPath", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
        }
    }*/

    public boolean getConfigured() {
        return configured;
    }
}