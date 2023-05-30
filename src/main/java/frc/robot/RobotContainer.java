package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.ElevatorVoltageCommand;
import frc.robot.Commands.SwerveJoystickCommand;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.ElevatorConstants;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Subsystems.IntakeSubsystem.GameElement;
import frc.robot.Subsystems.IntakeSubsystem.WheelDirection;
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

    //ElevatorVoltageCommand elevatorVoltageCommand = new ElevatorVoltageCommand(elevatorSubsystem);

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

        //elevatorSubsystem.setDefaultCommand(new ElevatorVoltageCommand(elevatorSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        configured = true;

        new JoystickButton(driverJoystick, 6).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredElevatorPosition(ElevatorSubsystem.ElevatorPosition.LOW);
            elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeLow);
        }, elevatorSubsystem));

        /*new JoystickButton(driverJoystick, 6).onTrue(Commands.runOnce(() -> {
            if(Constants.ElevatorConstants.isHome || !Constants.ElevatorConstants.isScore) {
                elevatorSubsystem.setSetpoint(getPositionToScore()[0]);
                intakeSubsystem.setSetpoint(getPositionToScore()[1]);

                intakeSubsystem.currWheelDirection = WheelDirection.OUTTAKE;

                Constants.ElevatorConstants.isHome = false;
                Constants.ElevatorConstants.isScore = true;
            }
            else {
                elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);

                Constants.ElevatorConstants.isHome = true;
                Constants.ElevatorConstants.isScore = false;
            }
        },elevatorSubsystem, intakeSubsystem));*/

        new JoystickButton(driverJoystick, 5).onTrue(Commands.runOnce(() -> {
            if(Constants.ElevatorConstants.isHome) {
            elevatorSubsystem.setSetpoint(getPositionToIntake()[0]);
            intakeSubsystem.setSetpoint(getPositionToIntake()[1]);

            intakeSubsystem.currWheelDirection = WheelDirection.INTAKE;

            Constants.ElevatorConstants.isHome = false;
            Constants.ElevatorConstants.isScore = true;
            }
            else {
                elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);

                intakeSubsystem.currWheelDirection = WheelDirection.OFF;

                Constants.ElevatorConstants.isHome = true;
                Constants.ElevatorConstants.isScore = false;
            }
        }, elevatorSubsystem, intakeSubsystem));

        new JoystickButton(driver2Joystick, 2).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredElevatorPosition(ElevatorSubsystem.ElevatorPosition.LOW);
        }, elevatorSubsystem));

        new JoystickButton(driver2Joystick, 3).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredElevatorPosition(ElevatorSubsystem.ElevatorPosition.MID);
        }, elevatorSubsystem));

        new JoystickButton(driver2Joystick, 4).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredElevatorPosition(ElevatorSubsystem.ElevatorPosition.HIGH);
        }, elevatorSubsystem));

        new POVButton(driver2Joystick, 0).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.GROUND_CONE_UP);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CONE);
        }, elevatorSubsystem, intakeSubsystem));

        new POVButton(driver2Joystick, 90).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.CONE_SUBSTATION);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CONE);
        }, elevatorSubsystem, intakeSubsystem));

        new POVButton(driver2Joystick, 180).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.GROUND_CONE_TIPPED);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CONE);
        }, elevatorSubsystem, intakeSubsystem));

        new POVButton(driver2Joystick, 270).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.GROUND_CUBE);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CUBE);
        }, elevatorSubsystem, intakeSubsystem));


       /* new JoystickButton(driverJoystick, 6)
                .onTrue(Commands.runOnce(() -> {
                    if(elevatorSubsystem.isElementInside) {
                        if(intakeSubsystem.getCurrGameElement() == IntakeSubsystem.GameElement.CONE) {
                            intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCone);
                        }
                    }
                }));*/

        new JoystickButton(driverJoystick, 1)
                .onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));    

        new JoystickButton(driverJoystick, 4).onTrue(Commands.runOnce(() -> {
            DriveConstants.kFieldCentric = !DriveConstants.kFieldCentric;
        }));


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

    public double[] getPositionToScore() {
        if(intakeSubsystem.getCurrGameElement() == IntakeSubsystem.GameElement.CONE) {
            if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.HIGH) {
                return new double[] { Constants.ElevatorConstants.kElevatorPosConeHigh, Constants.IntakeConstants.kPivotAngleRadScoreCone};
            } else
                if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.MID) {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosConeMid, Constants.IntakeConstants.kPivotAngleRadScoreCone};
                }
                else {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosConeLow, Constants.IntakeConstants.kPivotAngleRadScoreCone};
                }
        } else {
            if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.HIGH) {
                return new double[] { Constants.ElevatorConstants.kElevatorPosConeHigh, Constants.IntakeConstants.kPivotAngleRadScoreCube};
            } else
                if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.MID) {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosConeMid, Constants.IntakeConstants.kPivotAngleRadScoreCube};
                }
                else {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosConeLow, Constants.IntakeConstants.kPivotAngleRadScoreCube};
                }
        }
    }

    public double[] getPositionToIntake() {
        if(elevatorSubsystem.getDesiredIntakeGameElement() == ElevatorSubsystem.IntakeGameElement.GROUND_CONE_UP) {
            return new double[] {ElevatorConstants.kElevatorPosIntakeConeUp, IntakeConstants.kPivotAngleRadIntakeConeUp};
        }
        else if(elevatorSubsystem.getDesiredIntakeGameElement() == ElevatorSubsystem.IntakeGameElement.GROUND_CONE_TIPPED) {
            return new double[] {ElevatorConstants.kElevatorPosIntakeConeTipped, IntakeConstants.kPivotAngleRadIntakeConeTipped};
        }
        else if(elevatorSubsystem.getDesiredIntakeGameElement() == ElevatorSubsystem.IntakeGameElement.CONE_SUBSTATION) {
            return new double[] {ElevatorConstants.kElevatorPosIntakeConeSubstation, IntakeConstants.kPivotAngleRadIntakeConeSubstation};
        }
        else
            return new double[] {ElevatorConstants.kElevatorPosIntakeCubeGround, IntakeConstants.kPivotAngleRadIntakeCubeGround};
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
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
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
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
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
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
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
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
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
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
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
                    intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
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