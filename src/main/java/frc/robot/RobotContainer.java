package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.AutoFollowingCommand;
import frc.robot.Commands.AutoRetractCommand;
import frc.robot.Commands.AutoTrackingCommand;
import frc.robot.Commands.ElevatorManualControl;
import frc.robot.Commands.ParkCommand;
import frc.robot.Commands.SwerveJoystickCommand;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.ElevatorConstants;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Subsystems.ElevatorSubsystem.IntakeGameElement;
import frc.robot.Subsystems.IntakeSubsystem.GameElement;
import frc.robot.Subsystems.IntakeSubsystem.WheelDirection;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.CamMode;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.LEDState;
import frc.robot.Util.VisionTarget;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final LimelightSubsystem limeLightSubsystem = new LimelightSubsystem("tx", "ty", "ta");
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick driver2Joystick = new Joystick(OIConstants.kDriver2ControllerPort);
    private boolean configured = false;
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final SendableChooser autoStartSideChooser = new SendableChooser<String>();
    private final SendableChooser autoRoutineChooser = new SendableChooser<String>();

    public RobotContainer() {
        autoStartSideChooser.addOption("Red", "BlueSide");
        autoStartSideChooser.addOption("Blue", "RedSide");

        autoRoutineChooser.addOption("Mid", "AutoMid");
        autoRoutineChooser.addOption("Cable Cover", "AutoCableCover");
        autoRoutineChooser.addOption("Open", "AutoOpen");

        SmartDashboard.putData(autoStartSideChooser);
        SmartDashboard.putData(autoRoutineChooser);
        //SmartDashboard.putBoolean("isHome", ElevatorSubsystem.isHome);
        //SmartDashboard.putBoolean("isScore", ElevatorSubsystem.isScore);
    }

    public void teleOpInit() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> DriveConstants.kFieldCentric));

        swerveSubsystem.setDriveMotorsIdleMode(IdleMode.kBrake);

        //swerveSubsystem.setDefaultCommand(new AutoTrackingCommand(swerveSubsystem, limeLightSubsystem));

        //elevatorSubsystem.setDefaultCommand(new AutoRetractCommand(intakeSubsystem, elevatorSubsystem, ledSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        configured = true;

        new JoystickButton(driverJoystick, 6).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.removeDefaultCommand();
            if(ElevatorSubsystem.isHome || !ElevatorSubsystem.isScore) {
                DriveConstants.slowMode = true;
                elevatorSubsystem.setSetpoint(getPositionToScore()[0]);
                
                if(elevatorSubsystem.getDesiredElevatorPosition() !=  ElevatorSubsystem.ElevatorPosition.MID) {
                new Thread(() -> {
                    try {
                        Thread.sleep(450);
                        intakeSubsystem.setSetpoint(getPositionToScore()[1]); 
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();
            } else {
                new Thread(() -> {
                    try {
                        Thread.sleep(350);
                        intakeSubsystem.setSetpoint(getPositionToScore()[1]); 
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();
            }

                ElevatorSubsystem.isHome = false;
                ElevatorSubsystem.isScore = true;
            }
            else {
                new Thread(() -> {
                    try {
                        if(intakeSubsystem.getCurrGameElement() == IntakeSubsystem.GameElement.CONE) {
                    intakeSubsystem.setWheelDirection(WheelDirection.INTAKE);
                }
                else {
                    intakeSubsystem.setWheelDirection(WheelDirection.OUTTAKE);
                }

                        intakeSubsystem.updateWheelPower();

                        Thread.sleep(500);

                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);

                        intakeSubsystem.setWheelDirection(WheelDirection.OFF);

                        DriveConstants.slowMode = false;
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();

                ElevatorSubsystem.isHome = true;
                ElevatorSubsystem.isScore = false;
            }
        },elevatorSubsystem, intakeSubsystem));

        new JoystickButton(driverJoystick, 5).onTrue(Commands.runOnce(() -> {
            if(ElevatorSubsystem.isHome) {
            elevatorSubsystem.setDefaultCommand(new AutoRetractCommand(intakeSubsystem, elevatorSubsystem, ledSubsystem));
            elevatorSubsystem.setSetpoint(getPositionToIntake()[0]);

            if (elevatorSubsystem.getDesiredIntakeGameElement() != ElevatorSubsystem.IntakeGameElement.CONE_SUBSTATION && elevatorSubsystem.getDesiredIntakeGameElement() != ElevatorSubsystem.IntakeGameElement.GROUND_CONE_TIPPED) {
                new Thread(() -> {
                    try {
                        Thread.sleep(350);
                        intakeSubsystem.setSetpoint(getPositionToIntake()[1]); 
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();
            } else if(elevatorSubsystem.getDesiredIntakeGameElement() == ElevatorSubsystem.IntakeGameElement.GROUND_CONE_TIPPED) {
                new Thread(() -> {
                    try {
                        Thread.sleep(200);
                        intakeSubsystem.setSetpoint(getPositionToIntake()[1]); 
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();
            }
            else {
                intakeSubsystem.setSetpoint(getPositionToIntake()[1]);
            }

            if (intakeSubsystem.getCurrGameElement() == GameElement.CUBE) {
                intakeSubsystem.setWheelDirection(WheelDirection.INTAKE);
            } else {
                intakeSubsystem.setWheelDirection(WheelDirection.OUTTAKE);
            }
            
            ElevatorSubsystem.isHome = false;
            ElevatorSubsystem.isScore = false;
            }
            else {
                elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);

                intakeSubsystem.setWheelDirection(WheelDirection.OFF);

                DriveConstants.slowMode = false;

                ElevatorSubsystem.isHome = true;
                ElevatorSubsystem.isScore = false;
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

        new JoystickButton(driverJoystick, 2 ).onTrue(Commands.runOnce(() -> {
            intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);
        }, intakeSubsystem));

        new JoystickButton(driverJoystick, 3).onTrue(Commands.runOnce(() -> {
            intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
        }, intakeSubsystem));

        new JoystickButton(driverJoystick, 4).onTrue(Commands.runOnce(() -> {
            intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
        },intakeSubsystem));

        new POVButton(driver2Joystick, 0).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.GROUND_CONE_UP);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CONE);
            ledSubsystem.setColor(LEDSubsystem.Colors.ORANGE.r, LEDSubsystem.Colors.ORANGE.g, LEDSubsystem.Colors.ORANGE.b);
            ledSubsystem.setLEDColour(LEDSubsystem.Colors.ORANGE);
        }, elevatorSubsystem, intakeSubsystem));

        new POVButton(driver2Joystick, 90).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.CONE_SUBSTATION);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CONE);
            ledSubsystem.setColor(LEDSubsystem.Colors.ORANGE.r, LEDSubsystem.Colors.ORANGE.g, LEDSubsystem.Colors.ORANGE.b);
            ledSubsystem.setLEDColour(LEDSubsystem.Colors.ORANGE);
        }, elevatorSubsystem, intakeSubsystem));

        new POVButton(driver2Joystick, 180).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.GROUND_CONE_TIPPED);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CONE);
            ledSubsystem.setColor(LEDSubsystem.Colors.ORANGE.r, LEDSubsystem.Colors.ORANGE.g, LEDSubsystem.Colors.ORANGE.b);
            ledSubsystem.setLEDColour(LEDSubsystem.Colors.ORANGE);
        }, elevatorSubsystem, intakeSubsystem));

        new POVButton(driver2Joystick, 270).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.setDesiredIntakeGameElement(ElevatorSubsystem.IntakeGameElement.GROUND_CUBE);
            intakeSubsystem.setCurrGameElement(IntakeSubsystem.GameElement.CUBE);
            ledSubsystem.setColor(LEDSubsystem.Colors.PURPLE.r, LEDSubsystem.Colors.PURPLE.g, LEDSubsystem.Colors.PURPLE.b);
            ledSubsystem.setLEDColour(LEDSubsystem.Colors.PURPLE);
        }, elevatorSubsystem, intakeSubsystem)); 

        new JoystickButton(driverJoystick, 1)
                .onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));    

        new JoystickButton(driverJoystick, 10).onTrue(Commands.runOnce(() -> {
            DriveConstants.kFieldCentric = !DriveConstants.kFieldCentric;
        }));
        
        new JoystickButton(driverJoystick, 9).onTrue(Commands.runOnce(() -> {
            elevatorSubsystem.removeDefaultCommand();
        }, elevatorSubsystem));

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

        new JoystickButton(driver2Joystick, 10).onTrue(Commands.runOnce(() -> {
            swerveSubsystem.resetEncoders();
        }, swerveSubsystem));
    }

    public void resetTimers() {
        intakeSubsystem.resetTimer();
        elevatorSubsystem.resetTimer();
    }

    public double[] getPositionToScore() {
        if(intakeSubsystem.getCurrGameElement() == IntakeSubsystem.GameElement.CONE) {
            if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.HIGH) {
                return new double[] { Constants.ElevatorConstants.kElevatorPosConeHigh, Constants.IntakeConstants.kPivotAngleRadScoreConeHigh};
            } else
                if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.MID) {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosConeMid, Constants.IntakeConstants.kPivotAngleRadScoreConeMid};
                }
                else {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosConeLow, Constants.IntakeConstants.kPivotAngleRadScoreConeMid};
                }
        } else {
            if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.HIGH) {
                return new double[] { Constants.ElevatorConstants.kElevatorPosCubeHigh, Constants.IntakeConstants.kPivotAngleRadScoreCube};
            } else
                if (elevatorSubsystem.getDesiredElevatorPosition() ==  ElevatorSubsystem.ElevatorPosition.MID) {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosCubeMid, Constants.IntakeConstants.kPivotAngleRadScoreCube};
                }
                else {
                    return new double[] {Constants.ElevatorConstants.kElevatorPosCubeLow, Constants.IntakeConstants.kPivotAngleRadScoreCube};
                }
        }
    }

    public double[] getPositionToIntake() {
        if(elevatorSubsystem.getDesiredIntakeGameElement() == ElevatorSubsystem.IntakeGameElement.GROUND_CONE_UP) {
            return new double[] {ElevatorConstants.kElevatorPosIntakeConeDoubleSubstation, IntakeConstants.kPivotAngleRadIntakeConeDoubleSubstaion};
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
        AutoFollowingCommand autoFollowingCommand = new AutoFollowingCommand(
            swerveSubsystem, 
            limeLightSubsystem,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new VisionTarget(0, 0)   
        );

        return new SequentialCommandGroup(autoFollowingCommand);

        /*
        intakeSubsystem.isAuto = true;
        limeLightSubsystem.setLedMode(LEDState.ON);
        intakeSubsystem.currWheelDirection = WheelDirection.INTAKE;
        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);

        ParkCommand parkCommand = new ParkCommand(swerveSubsystem);

        if(autoStartSideChooser.getSelected() == "BlueSide") {
            if(autoRoutineChooser.getSelected() == "AutoOpen") {
                PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Blue 1+1 Open Exit Copy", new PathConstraints(2.0, 1.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Blue 1+1 Open Score 2", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Blue 1+1 Open Park 3", new PathConstraints(3.0, 3.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                    new SequentialCommandGroup(Commands.runOnce(() -> { 
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);
                        
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.53) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreConeHigh);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.2) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);
                        zeroTime = Timer.getFPGATimestamp();
                        intakeSubsystem.periodic();

                        while(Timer.getFPGATimestamp() - zeroTime <= 0.7) ;

                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        
                    }, elevatorSubsystem, intakeSubsystem)), 
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        new Thread(() -> {
                            try {
                                Thread.sleep(400);
                                elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }).start();
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);
                        intakeSubsystem.setSetpoint(Math.toRadians(0));

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.3) {
                            intakeSubsystem.periodic();
                        }
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    Commands.runOnce(() -> swerveSubsystem.setDriveMotorsIdleMode(IdleMode.kCoast)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                );
            }
            else if(autoRoutineChooser.getSelected() == "AutoMid") {
                PathPlannerTrajectory exitComunityTraj = PathPlanner.loadPath("Blue 1+1 Open Park", new PathConstraints(2, 2));
                PathPlannerTrajectory parkTraj = PathPlanner.loadPath("Blue 1+1 CableCover Park", new PathConstraints(2, 2));

                Command exitComTraj = swerveSubsystem.followTrajectoryCommand(exitComunityTraj, true);
                Command parkTrajectory = swerveSubsystem.followTrajectoryCommand(parkTraj, false);

                return new SequentialCommandGroup(new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.INTAKE);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            intakeSubsystem.periodic();
                            elevatorSubsystem.periodic();
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
                    }, elevatorSubsystem, intakeSubsystem),
                    exitComTraj, 

                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),

                    Commands.runOnce(() -> {
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 2.0) {
                            intakeSubsystem.periodic();
                            elevatorSubsystem.periodic();
                        }
                    }),

                    parkTrajectory
                ),
                parkCommand,
                Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                );
            }
            else {
                PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Blue 1+1 CableCover Exit Cone 1", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Blue 1+1 CableCover Score Cone", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Blue 1+1 CableCover Park Cone", new PathConstraints(3.0, 3.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);
                
                return new SequentialCommandGroup(
                new SequentialCommandGroup(Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.53) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreConeHigh);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.2) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.7) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        
                    }, elevatorSubsystem, intakeSubsystem)), 
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        new Thread(() -> {
                            try {
                                Thread.sleep(350);
                                elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }).start();
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);
                        intakeSubsystem.setSetpoint(Math.toRadians(-1));

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.3) {
                            intakeSubsystem.periodic();
                        }
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    Commands.runOnce(() -> swerveSubsystem.setDriveMotorsIdleMode(IdleMode.kCoast)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem) 
                );
            }
        }
        else {
            if(autoRoutineChooser.getSelected() == "AutoOpen") {

              PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Red 1+1 Open Exit Cone Copy Copy", new PathConstraints(2.0, 1.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Red 1+1 Open Score Cone Copy Copy", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Red 1+1 Open Park Cone 1 Copy Copy", new PathConstraints(4.0, 4.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                    new SequentialCommandGroup(Commands.runOnce(() -> { 
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);
                        
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.55) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreConeHigh);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.2) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);
                        zeroTime = Timer.getFPGATimestamp();
                        intakeSubsystem.periodic();

                        while(Timer.getFPGATimestamp() - zeroTime <= 0.7) ;

                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        
                    }, elevatorSubsystem, intakeSubsystem)), 
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        new Thread(() -> {
                            try {
                                Thread.sleep(400);
                                elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }).start();
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);
                        intakeSubsystem.setSetpoint(Math.toRadians(0));

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.3) {
                            intakeSubsystem.periodic();
                        }
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    Commands.runOnce(() -> swerveSubsystem.setDriveMotorsIdleMode(IdleMode.kCoast)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                );
            }
            else if(autoRoutineChooser.getSelected() == "AutoMid") {
                intakeSubsystem.setCurrGameElement(GameElement.CUBE);
                PathPlannerTrajectory exitComunityTraj = PathPlanner.loadPath("Blue 1+1 Open Park", new PathConstraints(2, 2));
                PathPlannerTrajectory parkTraj = PathPlanner.loadPath("Blue 1+1 CableCover Park", new PathConstraints(2, 2));

                Command exitComTraj = swerveSubsystem.followTrajectoryCommand(exitComunityTraj, true);
                Command parkTrajectory = swerveSubsystem.followTrajectoryCommand(parkTraj, false);

                return new SequentialCommandGroup(new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.INTAKE);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            intakeSubsystem.periodic();
                            elevatorSubsystem.periodic();
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
                    }, elevatorSubsystem, intakeSubsystem),
                    exitComTraj,

                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),

                    Commands.runOnce(() -> {
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 2.0) {
                            intakeSubsystem.periodic();
                            elevatorSubsystem.periodic();
                        }
                    }),

                     parkTrajectory
                ),
                parkCommand,
                Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                );
            }
            else {
                PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Red 1+1 CableCover Exit Cone 2 Copy", new PathConstraints(2.0, 1.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Red 1+1 CableCover Score Cone 4", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Red 1+1 CableCover Park Cone 4", new PathConstraints(3.0, 3.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                new SequentialCommandGroup(Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.53) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreConeHigh);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.2) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.7) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        
                    }, elevatorSubsystem, intakeSubsystem)), 
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        new Thread(() -> {
                            try {
                                Thread.sleep(350);
                                elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }).start();
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);
                        intakeSubsystem.setSetpoint(Math.toRadians(IntakeConstants.kPivotAngleRadIntakeCubeGround));

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.3) {
                            intakeSubsystem.periodic();
                        }
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    Commands.runOnce(() -> swerveSubsystem.setDriveMotorsIdleMode(IdleMode.kCoast)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                );
            }
        }*/
    }

    public String getAutoRoutine() {
        return autoRoutineChooser.getSelected().toString();
    }

    public boolean getConfigured() {
        return configured;
    }
}