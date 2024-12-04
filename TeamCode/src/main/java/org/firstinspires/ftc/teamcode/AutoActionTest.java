package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoActionTest")
public class AutoActionTest extends LinearOpMode {
    //@Config
    // i = intake, iC = intake claw, iW = intake wrist, iA = intake arm
    // d = delivery, dC = delivery claw, dCR = delivery claw right, dCL = delivery claw left, dW = delivery wrist
    public static double iCOpen;
    public static double iCClose;
    public static double iWTransferPos;
    public static double iWAlteredPos;
    public static double iWAlignmentPos;
    public static double iAUp;
    public static double iADown;
    public static double iAReady;
    public static double dCROpen;
    public static double dCRClose;
    public static double dCLOpen;
    public static double dCLClose;
    public static double dWTransfer;
    public static double dWDeliverBucket;
    public static double dWDeliverSpecimen;
    public static double dWStartPos;
    public static int verticalSlidePos;
    public static int horizontalSlidePos;
   // public static int target;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor horizontalSlideMotor;
    private static DcMotor verticalSlideMotor;
    private Servo intakeClaw;
    private Servo intakeWrist;
    private Servo intakeArm;
    private Servo deliveryWrist;
    private Servo deliveryClawR;
    private Servo deliveryClawL;
    public boolean GoNextAction = true;
    public boolean actionRunning = false;
    /* public class ScoreMove {
        public class moveSlide implements Action {
            verticalSlideMotor.setTargetPosition(vertPos);
            verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if ((verticalSlideMotor.getTargetPosition() - vertPos) < 0) {
                while (verticalSlideMotor.getCurrentPosition() - 10 > verticalSlideMotor.getTargetPosition()) {
                    verticalSlideMotor.setPower(0.3);
                }
            } else if ((verticalSlideMotor.getTargetPosition() - vertPos) > 0) {
                while (verticalSlideMotor.getCurrentPosition() + 10 < verticalSlideMotor.getTargetPosition()) {
                    verticalSlideMotor.setPower(-0.3);
                }
            }
            //return new TodoAction();
            return (Math.abs(vertPos) - 10 < verticalSlideMotor.getTargetPosition()

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            })
        }
    }*/
    public class SlideMove {
       //int target;
        private final DcMotorEx motor;
        public SlideMove(HardwareMap hardwareMap) {
            motor= hardwareMap.get(DcMotorEx.class,"verticalSlideMotor");
            //GoNextAction = false;
        }
        public class SlideMoveAction implements Action{

            int i = 0;
            int close = 0;
            int b = 0;
            boolean done = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                /*if (!GoNextAction & actionRunning) {
                    b += 1;
                    telemetry.addData("Waiting",b);
                    telemetry.update();
                    return true;
                }*/
                actionRunning = true;
                if (!done) {
                    //motor.setTargetPosition(target);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if ((verticalSlideMotor.getTargetPosition() - verticalSlideMotor.getCurrentPosition()) < 0) {
                        motor.setVelocity(-400);
                    } else if ((verticalSlideMotor.getTargetPosition() - verticalSlideMotor.getCurrentPosition()) > 0) {
                        motor.setVelocity(400);
                    }
                            done = true;
                }
                i += 1;
                //while (close > 0) {
                    double VertPos = motor.getCurrentPosition();
                    telemetryPacket.put("VertPos", VertPos);
                     close = Math.abs(verticalSlideMotor.getTargetPosition() - verticalSlideMotor.getCurrentPosition()) - 10;
                    telemetry.addData("Power", verticalSlideMotor.getPower());
                    telemetry.addData("TargetPosition", verticalSlideMotor.getTargetPosition());
                    telemetry.addData("Position", verticalSlideMotor.getCurrentPosition());
                    telemetry.addData("CloseVariable", close);
                    telemetry.addData("Cycles", i);
                    sleep(1000);
                    telemetry.update();
                //}
                /*if (i < 5) {
                    return true;
                }
                GoNextAction = true;
                actionRunning = false;*/
                return close > 0;
            }
        }

       public Action slideMoveAction(int target) {
            verticalSlideMotor.setTargetPosition(target);
            return new SlideMoveAction();
        }
   }
    public AutoActionTest() {
    }

    //Initialization code?

    @Override
    public void runOpMode() throws InterruptedException {
        //Motor and Servo setup w/ names

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

        //Initialization Code
        SlideMove slideMove = new SlideMove(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Actions.runBlocking(new SequentialAction(
                        slideMove.slideMoveAction(-1000),
                        slideMove.slideMoveAction(-300)
                ));
                Actions.runBlocking(slideMove.slideMoveAction(-1000));
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
        verticalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
    }
}
