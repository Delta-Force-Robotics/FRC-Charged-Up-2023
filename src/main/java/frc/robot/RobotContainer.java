package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
import frc.robot.Commands.AutoRetractCommand;
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
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Subsystems.ElevatorSubsystem.IntakeGameElement;
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

    public RobotContainer() {
        autoStartSideChooser.addOption("Blue", "BlueSide");
        autoStartSideChooser.addOption("Red", "RedSide");

        autoRoutineChooser.addOption("Mid", "AutoMid");
        autoRoutineChooser.addOption("Cable Cover", "AutoCableCover");
        autoRoutineChooser.addOption("Open", "AutoOpen");

        SmartDashboard.putData(autoStartSideChooser);
        SmartDashboard.putData(autoRoutineChooser);
        SmartDashboard.putBoolean("isHome", Constants.ElevatorConstants.isHome);
        SmartDashboard.putBoolean("isScore", Constants.ElevatorConstants.isScore);
    }

    public void teleOpInit() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> DriveConstants.kFieldCentric));

        elevatorSubsystem.setDefaultCommand(new AutoRetractCommand(intakeSubsystem, elevatorSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        configured = true;

        /*new JoystickButton(driverJoystick, 6).onTrue(Commands.runOnce(() -> {
            //elevatorSubsystem.setSetpoint(0.046);
            //intakeSubsystem.setSetpoint(Math.toRadians(-128));
            intakeSubsystem.setSetpoint(Math.toRadians(75));
            //intakeSubsystem.currWheelDirection = WheelDirection.INTAKE;
        }, intakeSubsystem));

        new JoystickButton(driverJoystick, 5).onTrue(Commands.runOnce(() -> {
            //elevatorSubsystem.setSetpoint(0);
            intakeSubsystem.setSetpoint(Math.toRadians(15));
            //intakeSubsystem.currWheelDirection = WheelDirection.OFF;
        }, intakeSubsystem));*/

        new JoystickButton(driverJoystick, 6).onTrue(Commands.runOnce(() -> {
            if(Constants.ElevatorConstants.isHome || !Constants.ElevatorConstants.isScore) {
                elevatorSubsystem.setSetpoint(getPositionToScore()[0]);

                new Thread(() -> {
                    try {
                        Thread.sleep(500);
                        intakeSubsystem.setSetpoint(getPositionToScore()[1]); 
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();

                if(intakeSubsystem.currWheelDirection != WheelDirection.OFF) {
                    if (intakeSubsystem.getCurrGameElement() == GameElement.CUBE) {
                        intakeSubsystem.currWheelDirection = WheelDirection.OUTTAKE;
                    } else {
                        intakeSubsystem.currWheelDirection = WheelDirection.INTAKE;
                    }
                }

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
        },elevatorSubsystem, intakeSubsystem));

        new JoystickButton(driverJoystick, 5).onTrue(Commands.runOnce(() -> {
            if(Constants.ElevatorConstants.isHome) {
            elevatorSubsystem.setSetpoint(getPositionToIntake()[0]);

            if (elevatorSubsystem.getDesiredIntakeGameElement() != ElevatorSubsystem.IntakeGameElement.CONE_SUBSTATION) {
                new Thread(() -> {
                    try {
                        Thread.sleep(350);
                        intakeSubsystem.setSetpoint(getPositionToIntake()[1]); 
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();
            } else {
                intakeSubsystem.setSetpoint(getPositionToIntake()[1]);
            }

            if (intakeSubsystem.getCurrGameElement() == GameElement.CUBE) {
                intakeSubsystem.currWheelDirection = WheelDirection.OUTTAKE;
            } else {
                intakeSubsystem.currWheelDirection = WheelDirection.INTAKE;
            }
            
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
        intakeSubsystem.currWheelDirection = WheelDirection.INTAKE;
        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);

        ParkCommand parkCommand = new ParkCommand(swerveSubsystem);

        if(autoStartSideChooser.getSelected() == "BlueSide") {
            if(autoRoutineChooser.getSelected() == "AutoOpen") {
                PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Blue 1+1 Open Exit", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Blue 1+1 Open Score", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Blue 1+1 Open Park", new PathConstraints(3.0, 3.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
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
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadIntakeCubeGround);

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeMid);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem) 
                );
            }
            else if(autoRoutineChooser.getSelected() == "AutoMid") {
                PathPlannerTrajectory parkTrajectory = PathPlanner.loadPath("Blue 1+Park", new PathConstraints(2.0, 2.0));

                Command parkTraj = swerveSubsystem.followTrajectoryCommand(parkTrajectory, true);

                return new SequentialCommandGroup(new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                        
                        intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }

                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                    }, elevatorSubsystem, intakeSubsystem),
                    parkTraj
                ),
                parkCommand,
                Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                );
            }
            else {
                PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Blue 1+1 CableCover Exit", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Blue 1+1 CableCover Score", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Blue 1+1 CableCover Park", new PathConstraints(2.0, 2.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
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
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadIntakeCubeGround);

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeMid);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem) 
                );
            }
        }
        else {
            if(autoRoutineChooser.getSelected() == "AutoOpen") {

              /*PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Red 1+1 Open Exit Cone Copy", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Red 1+1 Open Score Cone Copy", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Red 1+1 Open Park Cone 1", new PathConstraints(4.0, 4.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                    new SequentialCommandGroup(Commands.runOnce(() -> { 
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosConeHigh);
                        
                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.4) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreConeHigh);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
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
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadIntakeCubeGround);

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem) 
                );*/

                PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Red 1+1 Open Exit", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Red 1+1 Open Score", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Red 1+1 Open Park", new PathConstraints(4.0, 4.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
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
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadIntakeCubeGround);

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeMid);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem) 
                );
            }
            else if(autoRoutineChooser.getSelected() == "AutoMid") {
                PathPlannerTrajectory parkTrajectory = PathPlanner.loadPath("Red 1+Park", new PathConstraints(2.0, 2.0));

                Command parkTraj = swerveSubsystem.followTrajectoryCommand(parkTrajectory, true);

                return new SequentialCommandGroup(new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                        
                        intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }

                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        intakeSubsystem.setWheelDirection(frc.robot.Subsystems.IntakeSubsystem.WheelDirection.OFF);
                    }, elevatorSubsystem, intakeSubsystem),
                    parkTraj
                ),
                parkCommand,
                Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                );
            }
            else {
                PathPlannerTrajectory ExitTrajectory = PathPlanner.loadPath("Red 1+1 CableCover Exit", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ScoreTrajectory = PathPlanner.loadPath("Red 1+1 CableCover Score", new PathConstraints(2.0, 2.0));
                PathPlannerTrajectory ParkTrajectory = PathPlanner.loadPath("Red 1+1 CableCover Park", new PathConstraints(2.0, 2.0));

                Command traj1 = swerveSubsystem.followTrajectoryCommand(ExitTrajectory, true);
                Command traj2 = swerveSubsystem.followTrajectoryCommand(ScoreTrajectory, false);
                Command traj3 = swerveSubsystem.followTrajectoryCommand(ParkTrajectory, false);

                return new SequentialCommandGroup(
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeHigh);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                            elevatorSubsystem.periodic();
                            intakeSubsystem.periodic();
                        }
                    
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
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
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OUTTAKE);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadIntakeCubeGround);

                    }, intakeSubsystem),traj1),
                    new ParallelCommandGroup(Commands.runOnce(() -> {
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosCubeMid);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadScoreCube);
                        intakeSubsystem.setRampRate(0);

                    },intakeSubsystem, elevatorSubsystem), traj2),
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem),
                    new SequentialCommandGroup(Commands.runOnce(() -> {
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.INTAKE);

                        double zeroTime = Timer.getFPGATimestamp();
                        while(Timer.getFPGATimestamp() - zeroTime <= 0.5) {
                            intakeSubsystem.periodic();
                        }

                        intakeSubsystem.setRampRate(0.5);
                        intakeSubsystem.setWheelDirection(IntakeSubsystem.WheelDirection.OFF);
                        elevatorSubsystem.setSetpoint(ElevatorConstants.kElevatorPosHome);
                        intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
                        
                    }, intakeSubsystem,elevatorSubsystem)),
                    traj3,
                    Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem) 
                );
            }
        }
    }

    public boolean getConfigured() {
        return configured;
    }
}