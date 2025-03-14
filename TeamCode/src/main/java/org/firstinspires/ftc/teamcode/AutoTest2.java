package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

@Config
@Autonomous(name = "AutoTest2", group = "Autonomous")
public class AutoTest2 extends LinearOpMode {
    //@Config
    // i = intake, iC = intake claw, iW = intake wrist, iA = intake arm
    // d = delivery, dC = delivery claw, dCR = delivery claw right, dCL = delivery claw left, dW = delivery wrist
    public static double iCOpen = 0.8;
    public static double iCClose = 0.55;
    public static double iCAlign = 0.57;
    public static double iWTransferPos = 0.84;
    public static double iWAlteredPos;
    public static double iWAlignmentPos = 0;
    public static double iAUp = 0;
    public static double iADown = 0.64;
    public static double iAReady = 0.5;
    public static double dCROpen = 0.65;
    public static double dCRClose = 0.35;
    public static double dCLOpen = 0.27;
    public static double dCLClose = 0.55;
    public static double dWTransfer = 1;
    public static double dWDeliverBucket = 0.4;
    public static double dWDeliverSpecimen = 0.3;
    public static double dWStartPos = 0.8;
    public static int verticalSlidePos;
    public static int horizontalSlidePos;
    public static double BucketDeliverWait = 2.0;
    public static double PickupWait = 1.7;

    public static double TransferWait = 0.5;
    public static double RRInitPosX = 30;
    public static double RRInitPosY = 60;
    public static double RRInitPosHeading = Math.PI;
    public static double Traj1SetTangent = 0;
    public static double  Traj1X = 55;
    public static double  Traj1Y = 55;
    public static double  Traj1Heading = Math.PI / 4;
    public static double  Traj1TurnToHeading = -3 *  Math.PI / 4;
    public static double Traj2InitX = 55;
    public static double Traj2InitY = 55;
    public static double Traj2InitHeading = -3 * Math.PI / 4;
    public static double  Traj2TurnToHeading = -Math.PI / 2;
    public static double  Traj2LineToY = 40;
    public static double  Traj2X = 50;
    public static double  Traj2Y = 40;
    public static double Traj3InitX = 50;
    public static double Traj3InitY = 40;
    public static double Traj3InitHeading = -Math.PI / 2;
    public static double Traj3SetTangent = Math.PI / 2;
    public static double Traj3X = 55;
    public static double Traj3Y = 55;
    public static double Traj3Heading = Math.PI / 4;
    public static double HorizontalExtensionTicks = 65;
    public static double HorizontalRetractionTicks = 5;
    public static int TransferDelay1 = 250;
    public static int TransferDelay2 = 500;
    public static int PickUpDelay1 = 300;
    public static int PickUpDelay2 = 800;
    public static boolean PickupWithContact = true;
    public static int ForwardTimeForPickup = 400;

    // public static int target;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor horizontalSlideMotor;
    private DcMotor verticalSlideMotor;
    private Servo intakeClaw;
    private Servo intakeWrist;
    private Servo intakeArm;
    private Servo deliveryWrist;
    private Servo deliveryClawR;
    private Servo deliveryClawL;
    //public boolean GoNextAction = true;
    //public boolean actionRunning = false;


    public class VerticalSlide {
        private DcMotorEx verticalSlideMotor;

        public VerticalSlide(HardwareMap hardwareMap) {
            verticalSlideMotor = hardwareMap.get(DcMotorEx.class, "verticalSlideMotor");
            verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            verticalSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SlideUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    verticalSlideMotor.setPower(-0.7);
                    initialized = true;
                }

                // checks lift's current position
                double vertpos = verticalSlideMotor.getCurrentPosition();
                telemetry.addData("liftPos", vertpos);
                telemetry.update();
                if (vertpos > -2950.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    verticalSlideMotor.setPower(-0.2);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action slideUp() {
            return new SlideUp();
        }
        public class SlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    verticalSlideMotor.setPower(0.4);
                    initialized = true;
                }

                // checks lift's current position
                double vertpos = verticalSlideMotor.getCurrentPosition();
                telemetry.addData("liftPos", vertpos);
                telemetry.update();
                if (vertpos <-50) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    verticalSlideMotor.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action slideDown() {
            return new SlideDown();
        }
    }
    public class HorizontalSlide {
        private DcMotorEx horizontalSlideMotor;

        public HorizontalSlide(HardwareMap hardwareMap) {
            horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, "horizontalSlideMotor");
            horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            horizontalSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SlideForward implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    horizontalSlideMotor.setPower(0.3);
                    initialized = true;
                }

                // checks lift's current position
                double horizpos = horizontalSlideMotor.getCurrentPosition();
                telemetry.addData("liftPos", horizpos);
                telemetry.update();
                if (horizpos < HorizontalExtensionTicks) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    horizontalSlideMotor.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action slideForward() {
            return new SlideForward();
        }
        public class SlideBack implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    horizontalSlideMotor.setPower(-0.3);
                    initialized = true;
                }

                // checks lift's current position
                double horizpos = horizontalSlideMotor.getCurrentPosition();
                telemetry.addData("liftPos", horizpos);
                telemetry.update();
                if (horizpos > HorizontalRetractionTicks) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    horizontalSlideMotor.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action slideBack() {
            return new SlideBack();
        }
    }

    public class DeliverySystem {
        private Servo dCL;
        private Servo dCR;
        private Servo dW;

        public DeliverySystem(HardwareMap hardwareMap) {
            dCL = hardwareMap.get(Servo.class, "deliveryClawL");
            dCR = hardwareMap.get(Servo.class, "deliveryClawR");
            dW = hardwareMap.get(Servo.class, "deliveryWrist");
        }
        public class CloseDeliveryClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dCL.setPosition(dCLClose);
                dCR.setPosition(dCRClose);
                return false;
            }
        }
        public Action closeDeliveryClaw() {
            return new CloseDeliveryClaw();
        }
        public class OpenDeliveryClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dCL.setPosition(dCLOpen);
                dCR.setPosition(dCROpen);
                return false;
            }
        }
        public Action openDeliveryClaw() {
            return new OpenDeliveryClaw();
        }
        public class DeliveryWristBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWDeliverBucket);
                return false;
            }
        }
        public Action deliveryWristBack() {
            return new DeliveryWristBack();
        }
        public class DeliveryWristTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWTransfer);
                return false;
            }
        }
        public Action deliveryWristTransfer() {
            return new DeliveryWristTransfer();
        }
        public class DeliveryWristInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWStartPos);
                return false;
            }
        }
        public Action deliveryWristInit() {
            return new DeliveryWristInit();
        }
        public class DeliverToBucket implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWDeliverBucket);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                dCL.setPosition(dCLOpen);
                                dCR.setPosition(dCROpen);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                dW.setPosition(dWTransfer);
                                                new Timer().schedule(
                                                        new java.util.TimerTask() {
                                                            @Override
                                                            public void run() {
                                                                dCL.setPosition(dCLClose);
                                                                dCR.setPosition(dCRClose);
                                                                /*new Timer().schedule(
                                                                        new java.util.TimerTask() {
                                                                            @Override
                                                                            public void run() {

                                                                            }
                                                                        },1500
                                                                );*/
                                                            }
                                                        },500
                                                );
                                            }
                                        },500
                                );
                            }
                        },500
                );
                return false;
            }
        }
        public Action deliverToBucket() {
            return new DeliverToBucket();
        }
    }
    public class IntakeSystem {
        private Servo iC;
        private Servo iW;
        private Servo iA;
        private Servo dCL;
        private Servo dCR;
        private Servo dW;
        public IntakeSystem(HardwareMap hardwareMap) {
            iC = hardwareMap.get(Servo.class, "intakeClaw");
            iW = hardwareMap.get(Servo.class, "intakeWrist");
            iA = hardwareMap.get(Servo.class, "intakeArm");
            dCL = hardwareMap.get(Servo.class, "deliveryClawL");
            dCR = hardwareMap.get(Servo.class, "deliveryClawR");
            dW = hardwareMap.get(Servo.class, "deliveryWrist");

        }
        public class CloseIntakeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iC.setPosition(iCClose);
                return false;
            }
        }
        public Action closeIntakeClaw() {
            return new CloseIntakeClaw();
        }
        public class OpenIntakeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iC.setPosition(iCOpen);
                return false;
            }
        }
        public Action openIntakeClaw() {
            return new OpenIntakeClaw();
        }
        public class IntakeWristCenter implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iW.setPosition(iWTransferPos);
                return false;
            }
        }
        public Action intakeWristCenter() {
            return new IntakeWristCenter();
        }
        public class IntakeWristAlignTask implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iW.setPosition(iWAlignmentPos);
                iC.setPosition(iCClose);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                iC.setPosition(iCAlign);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                iC.setPosition(iCClose);
                                                new Timer().schedule(
                                                        new java.util.TimerTask() {
                                                            @Override
                                                            public void run() {
                                                                iC.setPosition(iCAlign);
                                                                new Timer().schedule(
                                                                        new java.util.TimerTask() {
                                                                            @Override
                                                                            public void run() {
                                                                                iC.setPosition(iCClose);
                                                                                new Timer().schedule(
                                                                                        new java.util.TimerTask() {
                                                                                            @Override
                                                                                            public void run() {
                                                                                                iW.setPosition(iWTransferPos);
                                                                                            }
                                                                                        },100
                                                                                );
                                                                            }
                                                                        },300
                                                                );
                                                            }
                                                        },100
                                                );
                                            }
                                        },300
                                );
                            }
                        },500
                );
                return false;
            }
        }
        public Action intakeWristAlignTask() {
            return new IntakeWristAlignTask();
        }
        public class IntakeArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iA.setPosition(iAUp);
                return false;
            }
        }
        public Action intakeArmUp() {
            return new IntakeArmUp();
        }
        public class IntakeArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iA.setPosition(iADown);
                return false;
            }
        }
        public Action intakeArmDown() {
            return new IntakeArmDown();
        }

        public class TransferToDelivery implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWTransfer);
                dCL.setPosition(dCLOpen);
                dCR.setPosition(dCROpen);
                iW.setPosition(iWTransferPos);
                iA.setPosition(iAUp);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                dCR.setPosition(dCRClose);
                                dCL.setPosition(dCLClose);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                iC.setPosition(iCOpen);
                                            }
                                        },TransferDelay1
                                );
                            }
                        },TransferDelay2
                );
                return false;
            }
        }
        public Action transferToDelivery() {
            return new TransferToDelivery();
        }
        public class PickUpSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iC.setPosition(iCOpen);
                iA.setPosition(iADown);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                iC.setPosition(iCClose);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                if (!PickupWithContact) {
                                                    iA.setPosition(iAUp);
                                                }
                                                if (PickupWithContact) {
                                                    horizontalSlideMotor.setPower(0.3);
                                                    iA.setPosition(iAReady);
                                                    new Timer().schedule(
                                                            new java.util.TimerTask() {
                                                                @Override
                                                                public void run() {
                                                                    horizontalSlideMotor.setPower(0);
                                                                    iA.setPosition(iAUp);
                                                                    new Timer().schedule(
                                                                            new java.util.TimerTask() {
                                                                                @Override
                                                                                public void run() {
                                                                                    /*
                                                                                    iC.setPosition(iCClose);
                                                                                    new Timer().schedule(
                                                                                            new java.util.TimerTask() {
                                                                                                @Override
                                                                                                public void run() {
                                                                                                    iC.setPosition(iCClose);
                                                                                                }
                                                                                            }, 100
                                                                                    );*/
                                                                                }
                                                                            }, 500
                                                                    );
                                                                }
                                                            }, ForwardTimeForPickup
                                                    );
                                                }
                                            }
                                        },PickUpDelay1
                                );
                            }
                        },PickUpDelay2
                );
                return false;
            }
        }
        public Action pickUpSample() {
            return new PickUpSample();
        }
    }

    public AutoTest2() {
    }


    @Override

    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(RRInitPosX, RRInitPosY, RRInitPosHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder Trajectory1 = drive.actionBuilder(initialPose)
                .setTangent(Traj1SetTangent)
                .splineToConstantHeading(new Vector2d(Traj1X, Traj1Y), Traj1Heading)
                .turnTo(Traj1TurnToHeading);
        //Will wait for slide to go up & score

        TrajectoryActionBuilder Trajectory2 = drive.actionBuilder(new Pose2d(Traj2InitX,Traj2InitY,Traj2InitHeading))
                .turnTo(Traj2TurnToHeading)
                .lineToY(Traj2LineToY)
                .strafeToConstantHeading(new Vector2d(Traj2X, Traj2Y));
        //Will wait for horizontal slide to extend & claw to grab

        TrajectoryActionBuilder Trajectory3 = drive.actionBuilder(new Pose2d(Traj3InitX,Traj3InitY,Traj3InitHeading))
                .setTangent(Traj3SetTangent)
                .splineTo(new Vector2d(Traj3X, Traj3Y), Traj3Heading);
        TrajectoryActionBuilder WaitForBucket = drive.actionBuilder(initialPose)
                .waitSeconds(BucketDeliverWait);
        TrajectoryActionBuilder WaitForPickup = drive.actionBuilder(initialPose)
                .waitSeconds(PickupWait);
        TrajectoryActionBuilder WaitForTransfer = drive.actionBuilder(initialPose)
                .waitSeconds(TransferWait);


        Action Traj1 = Trajectory1.build();
        Action Traj2 = Trajectory2.build();
        Action Traj3 = Trajectory3.build();
        Action waitForBucket = WaitForBucket.build();
        Action waitForPickup = WaitForPickup.build();
        Action waitForTransfer = WaitForTransfer.build();

        TrajectoryActionBuilder TrajectoryTest = drive.actionBuilder(initialPose)
                .turnTo(0)
                .lineToX(20)
                .turnTo(Math.PI)
                .lineToX(30)
                .setTangent(Math.PI)
                .splineTo(new Vector2d(10,50),-Math.PI/2);

        Action TrajTest = TrajectoryTest.build();
        initMotors();
        initServos();
        //Initialization Code

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                VerticalSlide verticalSlide = new VerticalSlide(hardwareMap);
                HorizontalSlide horizontalSlide = new HorizontalSlide(hardwareMap);
                DeliverySystem deliverySystem = new DeliverySystem(hardwareMap);
                IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                verticalSlide.slideUp(),
                                Traj1
                        ),
                        verticalSlide.slideUp(),
                        deliverySystem.deliverToBucket(),
                        waitForBucket,
                        verticalSlide.slideDown(),
                        Traj2,
                        horizontalSlide.slideForward(),
                        intakeSystem.pickUpSample(),
                        waitForPickup,
                        horizontalSlide.slideBack(),
                        intakeSystem.transferToDelivery(),
                        waitForTransfer,
                        new ParallelAction(
                                verticalSlide.slideUp(),
                                Traj3
                        ),
                        verticalSlide.slideUp(),
                        deliverySystem.deliverToBucket(),
                        waitForBucket,
                        verticalSlide.slideDown()
                ));
                /*Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                        TrajTest,
                                verticalSlide.slideUp()
                        ),
                        verticalSlide.slideUp(),
                        deliverySystem.deliverToBucket(),
                        waitForBucket,
                        verticalSlide.slideDown()
                        ));*/
                sleep(30000);
            }
        }
    }

    private void initServos() {
        intakeClaw.setPosition(iCOpen);
        intakeWrist.setPosition(iWTransferPos);
        intakeArm.setPosition(iAUp);
        deliveryWrist.setPosition(dWStartPos);
        deliveryClawL.setPosition(dCLClose);
        deliveryClawR.setPosition(dCRClose);
    }
    private void initMotors() {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        horizontalSlideMotor = hardwareMap.get(DcMotor.class, "horizontalSlideMotor");
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "verticalSlideMotor");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        deliveryWrist = hardwareMap.get(Servo.class, "deliveryWrist");
        deliveryClawR = hardwareMap.get(Servo.class, "deliveryClawR");
        deliveryClawL = hardwareMap.get(Servo.class, "deliveryClawL");
        //Reset any nonzero encoder values
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Go to run_without_encoder, to be swapped out later (?)
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Setting other modes: direction and motor braking
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //verticalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
    }
}
