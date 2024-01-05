package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.Constants.Constants.PIVOT_SUS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.Commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.Commands.ToScoreCommand;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.Utils.CommandOpModeAuto;
import org.firstinspires.ftc.teamcode.Vision.HSVAutoPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class AutoRedFar3 extends CommandOpModeAuto {
    SampleMecanumDrive drive;
    GlisiereSubsystem glisiereSubsystem;
    IntakeSubsystem intakeSubsystem;
    ScoringSubsystem scoringSubsystem;
    private InstantCommand pressureOpen;
    private InstantCommand pressureClose;
    private InstantCommand bratUp;

    private InstantCommand glisieraUp;
    private InstantCommand glisieraDown;

    private SequentialCommandGroup toScoreSequence;
    private SequentialCommandGroup scoreSequence;

    private ConditionalCommand autoCommand;

    private Pose2d startPosition = new Pose2d(-40, -64, Math.toRadians(90));

    private TrajectorySequence MovLeftPlace;
    private TrajectorySequence MovCentruPlace;
    private TrajectorySequence MovRightPlace;
    private TrajectorySequence MovLeftMoveToStack;
    private TrajectorySequence MovCentruMoveToStack;
    private TrajectorySequence MovRightMoveToStack;
    private TrajectorySequence StackToBackboard1;
    private TrajectorySequence StackToBackboardLeft;
    private TrajectorySequence StackToBackboardCenter;
    private TrajectorySequence StackToBackboardRight;
    private TrajectorySequence BackboardToStackCenter0;
    private TrajectorySequence BackboardToStackCenter1;
    private TrajectorySequence BackboardToStackGuide;

    private TrajectorySequence BackboardToStackCenter02;
    private TrajectorySequence BackboardToStackCenter12;
    private TrajectorySequence BackboardToStackGuide2;



    private TrajectorySequence StackToBackboardCenterR2;
    private TrajectorySequence StackToBackboardR2;

    private TrajectorySequence StackToBackboardCenterR3;
    private TrajectorySequence StackToBackboardR3;


    private HSVAutoPipeline pipeline = new HSVAutoPipeline(2);
    private OpenCvCamera camera;

    SequentialCommandGroup autoLeft;
    SequentialCommandGroup autoCenter;
    SequentialCommandGroup autoRight;


    private TrajectorySequence SplineToBackboard1;
    private TrajectorySequence SplineToBackboard2;
    private TrajectorySequence SplineToBackboard3;


    @Override
    public void initialize() {
        //initOpenCV();

        glisiereSubsystem = new GlisiereSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        scoringSubsystem = new ScoringSubsystem(hardwareMap);

        scoringSubsystem.pressureClose();
        intakeSubsystem.dropdownUp();

        scoringSubsystem.setBratPos(0.25);
        scoringSubsystem.setPivot(PIVOT_SUS - 0.6);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setVision(false);

        MovCentruPlace = drive.trajectorySequenceBuilder(startPosition)
                .lineToLinearHeading(new Pose2d(-50, -15, Math.toRadians(180)))
                .build();
        MovCentruMoveToStack = drive.trajectorySequenceBuilder(MovCentruPlace.end())
                .lineToLinearHeading(new Pose2d(-55.5, 3, Math.toRadians(180)))
                .build();


        StackToBackboard1 = drive.trajectorySequenceBuilder(MovCentruMoveToStack.end())
                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
                .build();
        StackToBackboardCenter = drive.trajectorySequenceBuilder(StackToBackboard1.end())
                .lineToLinearHeading(new Pose2d(50, -50, Math.toRadians(180)))
                .build();



        BackboardToStackCenter0 = drive.trajectorySequenceBuilder(StackToBackboardCenter.end())
                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
                .build();

        BackboardToStackCenter1 = drive.trajectorySequenceBuilder(BackboardToStackCenter0.end())
                .lineToLinearHeading(new Pose2d(-61, -5, Math.toRadians(180)))
                .build();

        BackboardToStackGuide = drive.trajectorySequenceBuilder(BackboardToStackCenter1.end())
                .lineToLinearHeading(new Pose2d(-62.5, -3, Math.toRadians(180)))
                .build();

        StackToBackboardR2 = drive.trajectorySequenceBuilder(BackboardToStackGuide.end())
                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
                .build();

        StackToBackboardCenterR2 = drive.trajectorySequenceBuilder(StackToBackboardR2.end())
                .lineToLinearHeading(new Pose2d(50, -50, Math.toRadians(180)))
                .build();



//tura 3

        BackboardToStackCenter02 = drive.trajectorySequenceBuilder(StackToBackboardCenterR2.end())
                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
                .build();

        BackboardToStackCenter12 = drive.trajectorySequenceBuilder(BackboardToStackCenter02.end())
                .lineToLinearHeading(new Pose2d(-60.5, -6, Math.toRadians(180)))
                .build();

        BackboardToStackGuide2 = drive.trajectorySequenceBuilder(BackboardToStackCenter12.end())
                .lineToLinearHeading(new Pose2d(-60, -5, Math.toRadians(180)))
                .build();

        StackToBackboardR3 = drive.trajectorySequenceBuilder(BackboardToStackGuide2.end())
                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
                .build();

        StackToBackboardCenterR3 = drive.trajectorySequenceBuilder(StackToBackboardR3.end())
                .lineToLinearHeading(new Pose2d(50, -50, Math.toRadians(180)))
                .build();














        MovLeftPlace = drive.trajectorySequenceBuilder(startPosition)
                .lineToLinearHeading(new Pose2d(-45, -35, Math.toRadians(90)))
                .build();

        MovRightPlace = drive.trajectorySequenceBuilder(startPosition)
                .lineToLinearHeading(new Pose2d(-36, -33, Math.toRadians(180)))
                .build();

        MovLeftMoveToStack = drive.trajectorySequenceBuilder(MovLeftPlace.end())
                .splineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
                .build();

        MovRightMoveToStack = drive.trajectorySequenceBuilder(MovRightPlace.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
                .build();



        StackToBackboardLeft = drive.trajectorySequenceBuilder(StackToBackboard1.end())
                .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(180)))
                .build();



        StackToBackboardRight = drive.trajectorySequenceBuilder(StackToBackboard1.end())
                .lineToLinearHeading(new Pose2d(50, -40, Math.toRadians(180)))
                .build();

        pressureOpen = new InstantCommand(() -> {
            scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
            scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
            scoringSubsystem.pressureToggle = false;
        });

        pressureClose = new InstantCommand(() -> {
            scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
            scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
            scoringSubsystem.pressureToggle = true;
        });

        bratUp = new InstantCommand(() -> {
            scoringSubsystem.setPivot(0);
        });

        glisieraUp = new InstantCommand(() -> {
            glisiereSubsystem.setGlisiereFinalPosition(Constants.GLISIERA_UP);
        });

        glisieraDown = new InstantCommand(() -> {
            glisiereSubsystem.setGlisiereFinalPosition(Constants.GLISIERA_DOWN);
        });

        register(glisiereSubsystem);

        //caz left
        autoLeft = new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPoseEstimate(startPosition)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(100),

                                new InstantCommand(()-> {
                                    scoringSubsystem.setPivot(PIVOT_SUS);
                                    scoringSubsystem.setBratPos(1);
                                    glisiereSubsystem.setGlisiereFinalPosition(500);
                                    intakeSubsystem.dropdownToggle = false;
                                })
                        ),
                        new RoadRunnerCommand(drive, MovLeftPlace)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.setDropdown(0);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                }),

                new WaitCommand(50),

                new InstantCommand(()-> {
                    scoringSubsystem.pressureClose();
                }),

                new ParallelCommandGroup(

                        new SequentialCommandGroup(

                                new InstantCommand(() -> {
                                    scoringSubsystem.setBratPos(0.07);
                                }),

                                new WaitCommand(400),

                                new InstantCommand(() -> {
                                    scoringSubsystem.setPivot(0);
                                    glisiereSubsystem.setGlisiereFinalPosition(20);
                                }),

                                new InstantCommand(()->intakeSubsystem.setDropdown(0.05))
                        ),

                        new InstantCommand(() -> intakeSubsystem.runFwd()),
                        new RoadRunnerCommand(drive, MovLeftMoveToStack)
                ),

                new ParallelCommandGroup(
                        new InstantCommand(() -> scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS)),
                        new InstantCommand(() -> scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS)),
                        new InstantCommand(() -> intakeSubsystem.setDropdown(0.125))
                ),

                //TODO: PRIMA TURA
                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                }),

                //new WaitCommand(300),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.02);
                }),

                new WaitCommand(600),

                new InstantCommand(() -> {
                    scoringSubsystem.pressureClose();
                }),

                new WaitCommand(200),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.04);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> intakeSubsystem.end()),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, StackToBackboard1),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new InstantCommand(intakeSubsystem::runRvs)
                        )
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardCenter)
                ),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter0),

                        new SequentialCommandGroup(
                                new InstantCommand(()-> {
                                    scoringSubsystem.setBratPos(0.07);
                                }),

                                new WaitCommand(600),

                                new InstantCommand(() -> {
                                    glisiereSubsystem.setGlisiereFinalPosition(20);
                                    scoringSubsystem.setPivot(0);
                                })
                        )
                ),

                new WaitCommand(100),

                new InstantCommand(()->{

                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);

                    intakeSubsystem.setDropdown(0.135);
                    intakeSubsystem.runFwd();
                    glisiereSubsystem.setGlisiereFinalPosition(50);
                    scoringSubsystem.setBratPos(0.07);

                    intakeSubsystem.runFwd();
                    intakeSubsystem.setDropdown(0.05);
                }),

                new ParallelCommandGroup(

                        new RoadRunnerCommand(drive, BackboardToStackCenter1),

                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intakeSubsystem.setDropdown(0.16);
                                    scoringSubsystem.setBratPos(0.02);
                                }),

                                new WaitCommand(200),

                                new InstantCommand(()->{
                                    scoringSubsystem.setPivot(0);
                                })
                        )
                ),

                new WaitCommand(200),

                new RoadRunnerCommand(drive, BackboardToStackGuide),


                //TODO TURA 2
                new WaitCommand(700),

                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new WaitCommand(150),

                new InstantCommand(() ->{
                    scoringSubsystem.setBratPos(0.05);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                    intakeSubsystem.dropdownUp();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> intakeSubsystem.end()),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> intakeSubsystem.runRvs())
                        ),

                        new RoadRunnerCommand(drive, StackToBackboardR2)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardCenterR2)
                ),


                //TODO: TURA 3

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter02),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                                    scoringSubsystem.setBratPos(0.07);
                                }),
                                new WaitCommand(400),
                                new InstantCommand(()->{
                                    scoringSubsystem.setPivot(0);
                                    glisiereSubsystem.setGlisiereFinalPosition(50);
                                })
                        )
                ),

                new WaitCommand(200),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.07);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    intakeSubsystem.setDropdown(0.27);
                    intakeSubsystem.runFwd();
                    glisiereSubsystem.setGlisiereFinalPosition(50);
                }),


                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter12),
                        new InstantCommand(() -> {
                            scoringSubsystem.setBratPos(0.07);
                            glisiereSubsystem.setGlisiereFinalPosition(50);
                        })
                ),


                new InstantCommand(()->intakeSubsystem.setDropdown(0.27)),

                new RoadRunnerCommand(drive, BackboardToStackGuide2),

                new WaitCommand(100),

                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setBratPos(0.02);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.04);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                    intakeSubsystem.dropdownUp();
                }),

                new WaitCommand(100),

                new InstantCommand(() -> intakeSubsystem.end()),


                new ParallelCommandGroup(

                        new RoadRunnerCommand(drive, StackToBackboardR3),

                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(intakeSubsystem::runRvs)
                        )
                ),


                new InstantCommand(()-> {
                    intakeSubsystem.setDropdown(0);
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                    scoringSubsystem.setPivot(PIVOT_SUS-0.15);
                    scoringSubsystem.setBratPos(Constants.BRAT_SUS);
                }),

                new RoadRunnerCommand(drive, StackToBackboardCenterR3),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                    scoringSubsystem.setBratPos(0.07);
                }),

                new WaitCommand(100),

                new InstantCommand(()->{
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setPivot(0);
                })
        );


        //caz center
        autoCenter = new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPoseEstimate(startPosition)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(100),

                                new InstantCommand(()-> {
                                    scoringSubsystem.setPivot(PIVOT_SUS);
                                    scoringSubsystem.setBratPos(1);
                                    glisiereSubsystem.setGlisiereFinalPosition(500);
                                    intakeSubsystem.dropdownToggle = false;
                                })
                        ),
                        new RoadRunnerCommand(drive, MovCentruPlace)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.setDropdown(0);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                }),

                new WaitCommand(50),

                new InstantCommand(()-> {
                    scoringSubsystem.pressureClose();
                }),

                new ParallelCommandGroup(

                        new SequentialCommandGroup(

                                new InstantCommand(() -> {
                                    scoringSubsystem.setBratPos(0.07);
                                }),

                                new WaitCommand(400),

                                new InstantCommand(() -> {
                                    scoringSubsystem.setPivot(0);
                                    glisiereSubsystem.setGlisiereFinalPosition(20);
                                }),

                                new InstantCommand(()->intakeSubsystem.setDropdown(0.05))
                        ),

                        new InstantCommand(() -> intakeSubsystem.runFwd()),
                        new RoadRunnerCommand(drive, MovCentruMoveToStack)
                ),

                new ParallelCommandGroup(
                        new InstantCommand(() -> scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS)),
                        new InstantCommand(() -> scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS)),
                        new InstantCommand(() -> intakeSubsystem.setDropdown(0.125))
                ),

                //TODO: PRIMA TURA
                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                }),

                //new WaitCommand(300),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.02);
                }),

                new WaitCommand(600),

                new InstantCommand(() -> {
                    scoringSubsystem.pressureClose();
                }),

                new WaitCommand(200),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.04);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> intakeSubsystem.end()),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, StackToBackboard1),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new InstantCommand(intakeSubsystem::runRvs)
                        )
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardCenter)
                ),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter0),

                        new SequentialCommandGroup(
                            new InstantCommand(()-> {
                                scoringSubsystem.setBratPos(0.07);
                            }),

                            new WaitCommand(600),

                            new InstantCommand(() -> {
                                glisiereSubsystem.setGlisiereFinalPosition(20);
                                scoringSubsystem.setPivot(0);
                            })
                        )
                ),

                new WaitCommand(100),

                new InstantCommand(()->{

                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);

                    intakeSubsystem.setDropdown(0.135);
                    intakeSubsystem.runFwd();
                    glisiereSubsystem.setGlisiereFinalPosition(50);
                    scoringSubsystem.setBratPos(0.07);

                    intakeSubsystem.runFwd();
                    intakeSubsystem.setDropdown(0.05);
                }),

                new ParallelCommandGroup(

                    new RoadRunnerCommand(drive, BackboardToStackCenter1),

                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                intakeSubsystem.setDropdown(0.16);
                                scoringSubsystem.setBratPos(0.02);
                            }),

                            new WaitCommand(200),

                            new InstantCommand(()->{
                                scoringSubsystem.setPivot(0);
                            })
                    )
                ),

                new WaitCommand(200),

                new RoadRunnerCommand(drive, BackboardToStackGuide),


                //TODO TURA 2
                new WaitCommand(700),

                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new WaitCommand(150),

                new InstantCommand(() ->{
                    scoringSubsystem.setBratPos(0.05);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                    intakeSubsystem.dropdownUp();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> intakeSubsystem.end()),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> intakeSubsystem.runRvs())
                        ),

                        new RoadRunnerCommand(drive, StackToBackboardR2)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardCenterR2)
                ),


                //TODO: TURA 3

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter02),
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                                scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                                scoringSubsystem.setBratPos(0.07);
                            }),
                            new WaitCommand(400),
                            new InstantCommand(()->{
                                scoringSubsystem.setPivot(0);
                                glisiereSubsystem.setGlisiereFinalPosition(50);
                            })
                        )
                ),

                new WaitCommand(200),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.07);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    intakeSubsystem.setDropdown(0.27);
                    intakeSubsystem.runFwd();
                    glisiereSubsystem.setGlisiereFinalPosition(50);
                }),


                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter12),
                        new InstantCommand(() -> {
                            scoringSubsystem.setBratPos(0.07);
                            glisiereSubsystem.setGlisiereFinalPosition(50);
                        })
                ),


                new InstantCommand(()->intakeSubsystem.setDropdown(0.27)),

                new RoadRunnerCommand(drive, BackboardToStackGuide2),

                new WaitCommand(100),

                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setBratPos(0.02);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.04);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                    intakeSubsystem.dropdownUp();
                }),

                new WaitCommand(100),

                new InstantCommand(() -> intakeSubsystem.end()),


                new ParallelCommandGroup(

                        new RoadRunnerCommand(drive, StackToBackboardR3),

                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(intakeSubsystem::runRvs)
                        )
                ),


                new InstantCommand(()-> {
                    intakeSubsystem.setDropdown(0);
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                    scoringSubsystem.setPivot(PIVOT_SUS-0.15);
                    scoringSubsystem.setBratPos(Constants.BRAT_SUS);
                }),

                new RoadRunnerCommand(drive, StackToBackboardCenterR3),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                    scoringSubsystem.setBratPos(0.07);
                }),

                new WaitCommand(100),

                new InstantCommand(()->{
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setPivot(0);
                })
        );




        //caz right
        autoRight = new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPoseEstimate(startPosition)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(100),

                                new InstantCommand(()-> {
                                    scoringSubsystem.setPivot(PIVOT_SUS);
                                    scoringSubsystem.setBratPos(1);
                                    glisiereSubsystem.setGlisiereFinalPosition(500);
                                    intakeSubsystem.dropdownToggle = false;
                                })
                        ),
                        new RoadRunnerCommand(drive, MovRightPlace)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.setDropdown(0);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                }),

                new WaitCommand(50),

                new InstantCommand(()-> {
                    scoringSubsystem.pressureClose();
                }),

                new ParallelCommandGroup(

                        new SequentialCommandGroup(

                                new InstantCommand(() -> {
                                    scoringSubsystem.setBratPos(0.07);
                                }),

                                new WaitCommand(400),

                                new InstantCommand(() -> {
                                    scoringSubsystem.setPivot(0);
                                    glisiereSubsystem.setGlisiereFinalPosition(20);
                                }),

                                new InstantCommand(()->intakeSubsystem.setDropdown(0.05))
                        ),

                        new InstantCommand(() -> intakeSubsystem.runFwd()),
                        new RoadRunnerCommand(drive, MovRightMoveToStack)
                ),

                new ParallelCommandGroup(
                        new InstantCommand(() -> scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS)),
                        new InstantCommand(() -> scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS)),
                        new InstantCommand(() -> intakeSubsystem.setDropdown(0.125))
                ),

                //TODO: PRIMA TURA
                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                }),

                //new WaitCommand(300),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.02);
                }),

                new WaitCommand(600),

                new InstantCommand(() -> {
                    scoringSubsystem.pressureClose();
                }),

                new WaitCommand(200),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.04);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> intakeSubsystem.end()),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, StackToBackboard1),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new InstantCommand(intakeSubsystem::runRvs)
                        )
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardCenter)
                ),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter0),

                        new SequentialCommandGroup(
                                new InstantCommand(()-> {
                                    scoringSubsystem.setBratPos(0.07);
                                }),

                                new WaitCommand(600),

                                new InstantCommand(() -> {
                                    glisiereSubsystem.setGlisiereFinalPosition(20);
                                    scoringSubsystem.setPivot(0);
                                })
                        )
                ),

                new WaitCommand(100),

                new InstantCommand(()->{

                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);

                    intakeSubsystem.setDropdown(0.135);
                    intakeSubsystem.runFwd();
                    glisiereSubsystem.setGlisiereFinalPosition(50);
                    scoringSubsystem.setBratPos(0.07);

                    intakeSubsystem.runFwd();
                    intakeSubsystem.setDropdown(0.05);
                }),

                new ParallelCommandGroup(

                        new RoadRunnerCommand(drive, BackboardToStackCenter1),

                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intakeSubsystem.setDropdown(0.16);
                                    scoringSubsystem.setBratPos(0.02);
                                }),

                                new WaitCommand(200),

                                new InstantCommand(()->{
                                    scoringSubsystem.setPivot(0);
                                })
                        )
                ),

                new WaitCommand(200),

                new RoadRunnerCommand(drive, BackboardToStackGuide),


                //TODO TURA 2
                new WaitCommand(700),

                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new WaitCommand(150),

                new InstantCommand(() ->{
                    scoringSubsystem.setBratPos(0.05);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                    intakeSubsystem.dropdownUp();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> intakeSubsystem.end()),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> intakeSubsystem.runRvs())
                        ),

                        new RoadRunnerCommand(drive, StackToBackboardR2)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardCenterR2)
                ),


                //TODO: TURA 3

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter02),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                                    scoringSubsystem.setBratPos(0.07);
                                }),
                                new WaitCommand(400),
                                new InstantCommand(()->{
                                    scoringSubsystem.setPivot(0);
                                    glisiereSubsystem.setGlisiereFinalPosition(50);
                                })
                        )
                ),

                new WaitCommand(200),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.07);
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    intakeSubsystem.setDropdown(0.27);
                    intakeSubsystem.runFwd();
                    glisiereSubsystem.setGlisiereFinalPosition(50);
                }),


                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, BackboardToStackCenter12),
                        new InstantCommand(() -> {
                            scoringSubsystem.setBratPos(0.07);
                            glisiereSubsystem.setGlisiereFinalPosition(50);
                        })
                ),


                new InstantCommand(()->intakeSubsystem.setDropdown(0.27)),

                new RoadRunnerCommand(drive, BackboardToStackGuide2),

                new WaitCommand(100),

                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setBratPos(0.02);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.04);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                    intakeSubsystem.dropdownUp();
                }),

                new WaitCommand(100),

                new InstantCommand(() -> intakeSubsystem.end()),


                new ParallelCommandGroup(

                        new RoadRunnerCommand(drive, StackToBackboardR3),

                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(intakeSubsystem::runRvs)
                        )
                ),


                new InstantCommand(()-> {
                    intakeSubsystem.setDropdown(0);
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                    scoringSubsystem.setPivot(PIVOT_SUS-0.15);
                    scoringSubsystem.setBratPos(Constants.BRAT_SUS);
                }),

                new RoadRunnerCommand(drive, StackToBackboardCenterR3),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                    scoringSubsystem.pressureToggle = false;
                    intakeSubsystem.end();
                }),

                new WaitCommand(150),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                    scoringSubsystem.setBratPos(0.07);
                }),

                new WaitCommand(100),

                new InstantCommand(()->{
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    scoringSubsystem.setPivot(0);
                })
        );

        //1 == center, 2 == left, 3 == right default
        /*autoCommand = new ConditionalCommand(
                autoLeft,
                new ConditionalCommand(
                        autoRight,
                        autoCenter,
                        () -> pipeline.getCaz() == 3
                ),
                () -> pipeline.getCaz() == 2
        );*/
    }

    @Override
    public void runOnce() {
        autoCenter.schedule();

        new Thread(() -> camera.closeCameraDevice());
    }

    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {}
        });
    }
}

