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

@Autonomous(name = "AutoConcursBLUEFar")
public class AutoBlueFar extends CommandOpModeAuto {
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

    private int caz = 3;

    private Pose2d startPosition = new Pose2d(-40, 64, Math.toRadians(270));

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
    private TrajectorySequence StackToBackboardLeft2;
    private TrajectorySequence StackToBackboardLeft3;

    private TrajectorySequence StackToBackboardRight2;
    private TrajectorySequence StackToBackboardRight3;
    private TrajectorySequence StackToBackboardR3;


    private HSVAutoPipeline pipeline = new HSVAutoPipeline(2);
    private OpenCvCamera camera;

    SequentialCommandGroup autoLeft;
    SequentialCommandGroup autoCenter;
    SequentialCommandGroup autoRight;



    private TrajectorySequence BackboardToStackRight02;
    private TrajectorySequence BackboardToStackRight12;
    private TrajectorySequence StackToBackboard1Right;
    private TrajectorySequence BackboardToStackRight0;
    private TrajectorySequence BackboardToStackRight1;
    private TrajectorySequence BackboardToStackGuideRight;
    private TrajectorySequence StackToBackboardRightR2;
    private TrajectorySequence BackboardToStackGuide2Right;
    private TrajectorySequence StackToBackboardRightR3;
    private TrajectorySequence StackToBackboardRightR32;
    private TrajectorySequence ParkRight;
    private TrajectorySequence ParkCenter;
    private TrajectorySequence ParkLeft;
    private TrajectorySequence StackToBackboard1Left;
    private TrajectorySequence BackboardToStackLeft0;
    private TrajectorySequence BackboardToStackLeft1;
    private TrajectorySequence BackboardToStackGuideLeft;
    private TrajectorySequence StackToBackboardLeftR2;


    @Override
    public void initialize() {
        //initOpenCV();

        glisiereSubsystem = new GlisiereSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        scoringSubsystem = new ScoringSubsystem(hardwareMap);

        glisiereSubsystem.glisiereAutoToggle = 2;
        scoringSubsystem.pressureClose();
        intakeSubsystem.dropdownUp();


        glisiereSubsystem.glisieraDreapta.encoder.reset();
        glisiereSubsystem.glisieraStanga.encoder.reset();

        scoringSubsystem.setBratPos(0.25);
        scoringSubsystem.setPivot(PIVOT_SUS - 0.6);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setVision(false);





        MovCentruPlace = drive.trajectorySequenceBuilder(startPosition)
                .lineToLinearHeading(new Pose2d(-50, 15, Math.toRadians(180)))
                .build();
        MovCentruMoveToStack = drive.trajectorySequenceBuilder(MovCentruPlace.end())
                .lineToLinearHeading(new Pose2d(-55.5, 3, Math.toRadians(180)))
                .build();
        StackToBackboard1 = drive.trajectorySequenceBuilder(MovCentruMoveToStack.end())
                .lineToLinearHeading(new Pose2d(30, 7, Math.toRadians(180)))
                .build();
        StackToBackboardCenter = drive.trajectorySequenceBuilder(StackToBackboard1.end())
                .lineToLinearHeading(new Pose2d(50, 50, Math.toRadians(180)))
                .build();
        BackboardToStackCenter0 = drive.trajectorySequenceBuilder(StackToBackboardCenter.end())
                .lineToLinearHeading(new Pose2d(30, 7, Math.toRadians(180)))
                .build();
        BackboardToStackCenter1 = drive.trajectorySequenceBuilder(BackboardToStackCenter0.end())
                .lineToLinearHeading(new Pose2d(-61, 5, Math.toRadians(180)))
                .build();
        BackboardToStackGuide = drive.trajectorySequenceBuilder(BackboardToStackCenter1.end())
                .lineToLinearHeading(new Pose2d(-62.5, 3, Math.toRadians(180)))
                .build();
        StackToBackboardR2 = drive.trajectorySequenceBuilder(BackboardToStackGuide.end())
                .lineToLinearHeading(new Pose2d(30, 7, Math.toRadians(180)))
                .build();
        StackToBackboardCenterR2 = drive.trajectorySequenceBuilder(StackToBackboardR2.end())
                .lineToLinearHeading(new Pose2d(50, 50, Math.toRadians(180)))
                .build();
        ParkCenter = drive.trajectorySequenceBuilder(StackToBackboardCenterR2.end())
                .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(180)))
                .build();


//        BackboardToStackCenter02 = drive.trajectorySequenceBuilder(StackToBackboardCenterR2.end())
//                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
//                .build();
//        BackboardToStackCenter12 = drive.trajectorySequenceBuilder(BackboardToStackCenter02.end())
//                .lineToLinearHeading(new Pose2d(-60.5, -6, Math.toRadians(180)))
//                .build();
//        BackboardToStackGuide2 = drive.trajectorySequenceBuilder(BackboardToStackCenter12.end())
//                .lineToLinearHeading(new Pose2d(-60, -5, Math.toRadians(180)))
//                .build();
//        StackToBackboardR3 = drive.trajectorySequenceBuilder(BackboardToStackGuide2.end())
//                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
//                .build();
//        StackToBackboardCenterR3 = drive.trajectorySequenceBuilder(StackToBackboardR3.end())
//                .lineToLinearHeading(new Pose2d(50, -50, Math.toRadians(180)))
//                .build();




        MovLeftPlace = drive.trajectorySequenceBuilder(startPosition)
                .lineToLinearHeading(new Pose2d(-60, 30, Math.toRadians(180)))
                .build();
        MovLeftMoveToStack = drive.trajectorySequenceBuilder(MovLeftPlace.end())
                .lineToLinearHeading(new Pose2d(-59, 2, Math.toRadians(180)))
                .build();
        StackToBackboard1Left = drive.trajectorySequenceBuilder(MovLeftMoveToStack.end())
                .lineToLinearHeading(new Pose2d(30, 7, Math.toRadians(180)))
                .build();
        StackToBackboardLeft = drive.trajectorySequenceBuilder(StackToBackboard1Left.end())
                .lineToLinearHeading(new Pose2d(48, 35, Math.toRadians(180)))
                .build();
        BackboardToStackLeft0 = drive.trajectorySequenceBuilder(StackToBackboardLeft.end())
                .lineToLinearHeading(new Pose2d(30, 3, Math.toRadians(180)))
                .build();
        BackboardToStackLeft1 = drive.trajectorySequenceBuilder(BackboardToStackLeft0.end())
                .lineToLinearHeading(new Pose2d(-62, 3, Math.toRadians(180)))
                .build();
        BackboardToStackGuideLeft = drive.trajectorySequenceBuilder(BackboardToStackLeft1.end())
                .lineToLinearHeading(new Pose2d(-63.5, 4, Math.toRadians(180)))
                .build();
        StackToBackboardLeftR2 = drive.trajectorySequenceBuilder(BackboardToStackGuideLeft.end())
                .lineToLinearHeading(new Pose2d(30, 7, Math.toRadians(180)))
                .build();
        StackToBackboardLeft2 = drive.trajectorySequenceBuilder(StackToBackboardLeftR2.end())
                .lineToLinearHeading(new Pose2d(46, 30, Math.toRadians(180)))
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(StackToBackboardLeft2.end())
                .lineToLinearHeading(new Pose2d(50, 18, Math.toRadians(180)))
                .build();



        MovRightPlace = drive.trajectorySequenceBuilder(startPosition)
                .lineToLinearHeading(new Pose2d(-41.5, -30, Math.toRadians(180)))
                .build();
        MovRightMoveToStack = drive.trajectorySequenceBuilder(MovRightPlace.end())
                .lineToLinearHeading(new Pose2d(-56.5, 0, Math.toRadians(180)))
                .build();
        StackToBackboard1Right = drive.trajectorySequenceBuilder(MovRightMoveToStack.end())
                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
                .build();
        StackToBackboardRight = drive.trajectorySequenceBuilder(StackToBackboard1Right.end())
                .lineToLinearHeading(new Pose2d(48, -46, Math.toRadians(180)))
                .build();
        BackboardToStackRight0 = drive.trajectorySequenceBuilder(StackToBackboardRight.end())
                .lineToLinearHeading(new Pose2d(30, -3, Math.toRadians(180)))
                .build();
        BackboardToStackRight1 = drive.trajectorySequenceBuilder(BackboardToStackRight0.end())
                .lineToLinearHeading(new Pose2d(-62, -3, Math.toRadians(180)))
                .build();
        BackboardToStackGuideRight = drive.trajectorySequenceBuilder(BackboardToStackRight1.end())
                .lineToLinearHeading(new Pose2d(-63, -4, Math.toRadians(180)))
                .build();
        StackToBackboardRightR2 = drive.trajectorySequenceBuilder(BackboardToStackGuideRight.end())
                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
                .build();
        StackToBackboardRight2 = drive.trajectorySequenceBuilder(StackToBackboardRightR2.end())
                .lineToLinearHeading(new Pose2d(46, -50, Math.toRadians(180)))
                .build();
        ParkRight = drive.trajectorySequenceBuilder(StackToBackboardRight2.end())
                .lineToLinearHeading(new Pose2d(50, -60, Math.toRadians(180)))
                .build();



//        BackboardToStackRight02 = drive.trajectorySequenceBuilder(StackToBackboardRight2.end())
//                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
//                .build();
//        BackboardToStackRight12 = drive.trajectorySequenceBuilder(BackboardToStackRight02.end())
//                .lineToLinearHeading(new Pose2d(-60.5, -6, Math.toRadians(180)))
//                .build();
//        BackboardToStackGuide2Right = drive.trajectorySequenceBuilder(BackboardToStackRight12.end())
//                .lineToLinearHeading(new Pose2d(-60, -5, Math.toRadians(180)))
//                .build();
//        StackToBackboardRightR3 = drive.trajectorySequenceBuilder(BackboardToStackGuide2Right.end())
//                .lineToLinearHeading(new Pose2d(30, -7, Math.toRadians(180)))
//                .build();
//        StackToBackboardRightR32 = drive.trajectorySequenceBuilder(StackToBackboardRightR3.end())
//                .lineToLinearHeading(new Pose2d(45, -50, Math.toRadians(180)))
//                .build();


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






        //TODO caz left
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
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS+0.05);
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
                        new RoadRunnerCommand(drive, StackToBackboard1Left),
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
                        new RoadRunnerCommand(drive, StackToBackboardLeft)
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
                        new RoadRunnerCommand(drive, BackboardToStackLeft0),

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

                    intakeSubsystem.setDropdown(0.14);
                    intakeSubsystem.runFwd();
                    glisiereSubsystem.setGlisiereFinalPosition(50);
                    scoringSubsystem.setBratPos(0.07);
                    intakeSubsystem.setDropdown(0.05);
                }),

                new ParallelCommandGroup(

                        new RoadRunnerCommand(drive, BackboardToStackLeft1),

                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intakeSubsystem.setDropdown(0.165);
                                    scoringSubsystem.setBratPos(0.02);
                                }),

                                new WaitCommand(200),

                                new InstantCommand(()->{
                                    scoringSubsystem.setPivot(0);
                                })
                        )
                ),

                new WaitCommand(200),

                new RoadRunnerCommand(drive, BackboardToStackGuideLeft),


                //TODO TURA 2
                new InstantCommand(()->{
                    glisiereSubsystem.setGlisiereFinalPosition(0);
                    intakeSubsystem.setDropdown(0.17);
                }),

                new WaitCommand(700),

                new InstantCommand(() -> {
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

                        new RoadRunnerCommand(drive, StackToBackboardLeftR2)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardLeft2)
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
                        new RoadRunnerCommand(drive, ParkLeft),
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
                )
//
//                new WaitCommand(200),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setBratPos(0.07);
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
//                    intakeSubsystem.setDropdown(0.27);
//                    intakeSubsystem.runFwd();
//                    glisiereSubsystem.setGlisiereFinalPosition(50);
//                }),
//
//
//                new ParallelCommandGroup(
//                        new RoadRunnerCommand(drive, BackboardToStackCenter12),
//                        new InstantCommand(() -> {
//                            scoringSubsystem.setBratPos(0.07);
//                            glisiereSubsystem.setGlisiereFinalPosition(50);
//                        })
//                ),
//
//
//                new InstantCommand(()->intakeSubsystem.setDropdown(0.27)),
//
//                new RoadRunnerCommand(drive, BackboardToStackGuide2),
//
//                new WaitCommand(100),
//
//                new InstantCommand(() -> {
//                    glisiereSubsystem.setGlisiereFinalPosition(0);
//                    scoringSubsystem.setBratPos(0.02);
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setBratPos(0.04);
//                    glisiereSubsystem.setGlisiereFinalPosition(250);
//                    intakeSubsystem.dropdownUp();
//                }),
//
//                new WaitCommand(100),
//
//                new InstantCommand(() -> intakeSubsystem.end()),
//
//
//                new ParallelCommandGroup(
//
//                        new RoadRunnerCommand(drive, StackToBackboardR3),
//
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(intakeSubsystem::runRvs)
//                        )
//                ),
//
//
//                new InstantCommand(()-> {
//                    intakeSubsystem.setDropdown(0);
//                    intakeSubsystem.end();
//                    intakeSubsystem.dropdownUp();
//                    glisiereSubsystem.setGlisiereFinalPosition(1000);
//                    scoringSubsystem.setPivot(PIVOT_SUS-0.15);
//                    scoringSubsystem.setBratPos(Constants.BRAT_SUS);
//                }),
//
//                new RoadRunnerCommand(drive, StackToBackboardLeft3),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
//                    scoringSubsystem.pressureToggle = false;
//                    intakeSubsystem.end();
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
//                    scoringSubsystem.setBratPos(0.07);
//                }),
//
//                new WaitCommand(100),
//
//                new InstantCommand(()->{
//                    glisiereSubsystem.setGlisiereFinalPosition(0);
//                    scoringSubsystem.setPivot(0);
//                })
        );










        //TODO caz center
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
                        new RoadRunnerCommand(drive, ParkCenter),
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
                )

//                new WaitCommand(200),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setBratPos(0.07);
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
//                    intakeSubsystem.setDropdown(0.27);
//                    intakeSubsystem.runFwd();
//                    glisiereSubsystem.setGlisiereFinalPosition(50);
//                }),
//
//
//                new ParallelCommandGroup(
//                        new RoadRunnerCommand(drive, BackboardToStackCenter12),
//                        new InstantCommand(() -> {
//                            scoringSubsystem.setBratPos(0.07);
//                            glisiereSubsystem.setGlisiereFinalPosition(50);
//                        })
//                ),
//
//
//                new InstantCommand(()->intakeSubsystem.setDropdown(0.27)),
//
//                new RoadRunnerCommand(drive, BackboardToStackGuide2),
//
//                new WaitCommand(100),
//
//                new InstantCommand(() -> {
//                    glisiereSubsystem.setGlisiereFinalPosition(0);
//                    scoringSubsystem.setBratPos(0.02);
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setBratPos(0.04);
//                    glisiereSubsystem.setGlisiereFinalPosition(250);
//                    intakeSubsystem.dropdownUp();
//                }),
//
//                new WaitCommand(100),
//
//                new InstantCommand(() -> intakeSubsystem.end()),
//
//
//                new ParallelCommandGroup(
//
//                        new RoadRunnerCommand(drive, StackToBackboardR3),
//
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(intakeSubsystem::runRvs)
//                        )
//                ),
//
//
//                new InstantCommand(()-> {
//                    intakeSubsystem.setDropdown(0);
//                    intakeSubsystem.end();
//                    intakeSubsystem.dropdownUp();
//                    glisiereSubsystem.setGlisiereFinalPosition(1000);
//                    scoringSubsystem.setPivot(PIVOT_SUS-0.15);
//                    scoringSubsystem.setBratPos(Constants.BRAT_SUS);
//                }),
//
//                new RoadRunnerCommand(drive, StackToBackboardCenterR3),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
//                    scoringSubsystem.pressureToggle = false;
//                    intakeSubsystem.end();
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
//                    scoringSubsystem.setBratPos(0.07);
//                }),
//
//                new WaitCommand(100),
//
//                new InstantCommand(()->{
//                    glisiereSubsystem.setGlisiereFinalPosition(0);
//                    scoringSubsystem.setPivot(0);
//                })
        );








        //TODO caz right
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
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS+0.02);
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
                    glisiereSubsystem.setGlisiereFinalPosition(20);
                }),
                new WaitCommand(100),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.00);
                }),
                new WaitCommand(700),

                new InstantCommand(() -> {
                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                }),

                new WaitCommand(300),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.02);
                }),

                new WaitCommand(600),

                new InstantCommand(() -> {
                    scoringSubsystem.pressureClose();
                }),

                new WaitCommand(500),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.04);
                    glisiereSubsystem.setGlisiereFinalPosition(250);
                }),

                new WaitCommand(100),

                new InstantCommand(() -> intakeSubsystem.end()),

                new ParallelCommandGroup(
                        new RoadRunnerCommand(drive, StackToBackboard1Right),
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
                        new RoadRunnerCommand(drive, StackToBackboardRight)
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
                        new RoadRunnerCommand(drive, BackboardToStackRight0),

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

                        new RoadRunnerCommand(drive, BackboardToStackRight1),

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

                new RoadRunnerCommand(drive, BackboardToStackGuideRight),


                //TODO TURA 2
                new WaitCommand(500),

                new InstantCommand(() -> {
                    glisiereSubsystem.setGlisiereFinalPosition(20);
                }),
                new WaitCommand(100),

                new InstantCommand(() -> {
                    scoringSubsystem.setBratPos(0.00);
                }),
                new WaitCommand(700),

                new InstantCommand(() -> {
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

                        new RoadRunnerCommand(drive, StackToBackboardRightR2)
                ),

                new InstantCommand(() -> {
                    intakeSubsystem.end();
                    intakeSubsystem.dropdownUp();
                    glisiereSubsystem.setGlisiereFinalPosition(1000);
                }),

                new ParallelCommandGroup(
                        new ToScoreCommand(1000, PIVOT_SUS - 0.15,  0, scoringSubsystem, glisiereSubsystem),
                        new RoadRunnerCommand(drive, StackToBackboardRight2)
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
//                        new RoadRunnerCommand(drive, BackboardToStackRight02),
                        new RoadRunnerCommand(drive, ParkRight),
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
                )
//
//                new WaitCommand(200),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setBratPos(0.07);
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
//                    intakeSubsystem.setDropdown(0.27);
//                    intakeSubsystem.runFwd();
//                    glisiereSubsystem.setGlisiereFinalPosition(50);
//                }),
//
//
//                new ParallelCommandGroup(
//                        new RoadRunnerCommand(drive, BackboardToStackRight12),
//                        new InstantCommand(() -> {
//                            scoringSubsystem.setBratPos(0.07);
//                            glisiereSubsystem.setGlisiereFinalPosition(50);
//                        })
//                ),
//
//
//                new InstantCommand(()->intakeSubsystem.setDropdown(0.27)),
//
//                new RoadRunnerCommand(drive, BackboardToStackGuide2Right),
//
//                new WaitCommand(100),
//
//                new InstantCommand(() -> {
//                    glisiereSubsystem.setGlisiereFinalPosition(0);
//                    scoringSubsystem.setBratPos(0.02);
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setBratPos(0.04);
//                    glisiereSubsystem.setGlisiereFinalPosition(250);
//                    intakeSubsystem.dropdownUp();
//                }),
//
//                new WaitCommand(100),
//
//                new InstantCommand(() -> intakeSubsystem.end()),
//
//
//                new ParallelCommandGroup(
//
//                        new RoadRunnerCommand(drive, StackToBackboardRightR3),
//
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(intakeSubsystem::runRvs)
//                        )
//                ),
//
//
//                new InstantCommand(()-> {
//                    intakeSubsystem.setDropdown(0);
//                    intakeSubsystem.end();
//                    intakeSubsystem.dropdownUp();
//                    glisiereSubsystem.setGlisiereFinalPosition(1000);
//                    scoringSubsystem.setPivot(PIVOT_SUS-0.15);
//                    scoringSubsystem.setBratPos(Constants.BRAT_SUS);
//                }),
//
//                new RoadRunnerCommand(drive, StackToBackboardRightR32),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
//                    scoringSubsystem.pressureToggle = false;
//                    intakeSubsystem.end();
//                }),
//
//                new WaitCommand(150),
//
//                new InstantCommand(() -> {
//                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
//                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
//                    scoringSubsystem.setBratPos(0.07);
//                }),
//
//                new WaitCommand(100),
//
//                new InstantCommand(()->{
//                    glisiereSubsystem.setGlisiereFinalPosition(0);
//                    scoringSubsystem.setPivot(0);
//                })
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
        autoLeft.schedule();

        //1 == center default, 2 == left, 3 == right
//        autoCommand = new ConditionalCommand(
//                autoLeft,
//                new ConditionalCommand(
//                        autoRight,
//                        autoCenter,
//                        () -> /*pipeline.getCaz() == 3*/ caz==3
//                ),
//                () -> /*pipeline.getCaz() == 2*/ caz==2
//        );


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

