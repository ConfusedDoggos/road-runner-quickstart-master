package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "intoTheDeepTeleAS2")
public class intoTheDeepTeleAS2 extends LinearOpMode {
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


    float horizontal;
    float vertical;
    float pivot;
    int vertPos;
    int horizPos;
    double targetPos;
    double intakeWristTargetPos;
    boolean freeWristRotate;
    boolean isOpen;
    double downCounter;

    public intoTheDeepTeleAS2() {
    }


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
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


        initMotorModes();
        initServoModes();
        //Initialization: All servos should be set to default position

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                moveVariables();
                speedSettings();
                mecanumMath();
                if (gamepad1.a) {
                    intakeClaw.setPosition(0.8);
                    /*if (!freeWristRotate) {
                        deliveryClawR.setPosition(0.5);
                        deliveryClawL.setPosition(0.3);
                    }*/
                    isOpen = true;
                    //intake claw open
                } else if (gamepad1.b) {
                    //intake claw close
                    intakeClaw.setPosition(0.55);
                    isOpen = false;
                }
                if (gamepad1.left_stick_button) {
                    Timer transferTimer = new Timer();
                    class transfer extends TimerTask {
                        int taskNumber = 0;

                        public void run() {
                            if (taskNumber == 1) {
                                deliveryClawR.setPosition(0.35);
                                deliveryClawL.setPosition(0.27);
                            } else if (taskNumber == 2) {
                                intakeClaw.setPosition(0.8);
                            }
                            taskNumber += 1;
                            if (taskNumber > 2) {
                                transferTimer.cancel();
                            }
                        }
                    }

                    TimerTask intakeTask = new transfer();
                    transferTimer.schedule(intakeTask, 0, 200);
                }
                if (gamepad1.x) {
                    deliveryClawR.setPosition(0.65);
                    deliveryClawL.setPosition(0.27);
                } else if (gamepad1.y) {
                    deliveryClawR.setPosition(0.35);
                    deliveryClawL.setPosition(0.55);
                }
                if (gamepad1.dpad_left) {
                    deliveryWrist.setPosition(0.55);
                } else if (gamepad1.dpad_right) {
                    deliveryWrist.setPosition(1);
                }
                if (gamepad1.right_bumper) {
                    deliveryWrist.setPosition(0.4);
                }
                /*if (gamepad2.dpad_up) {
                    targetPos += 0.001;
                } else if (gamepad2.dpad_down) {
                    targetPos -= 0.001;
                }
                if (gamepad2.left_trigger > 0.5) {
                    intakeClaw.setPosition(targetPos);
                }*/

                if (gamepad1.dpad_up/* & horizontalSlideMotor.getCurrentPosition() < 300*/) {
                    downCounter = 0;
                    intakeArm.setPosition(0);
                    freeWristRotate = false;
                    deliveryWrist.setPosition(1);
                    deliveryClawR.setPosition(0.4);
                    deliveryClawL.setPosition(0.45);
                    intakeWrist.setPosition(0.84);
                    Timer alignmentTimer = new Timer();
                    class alignment extends TimerTask {
                        int taskNumber = 0;

                        public void run() {
                            if (taskNumber == 1) {
                                intakeWrist.setPosition(0);
                            } else if (taskNumber == 2) {
                                intakeClaw.setPosition(0.57);
                            } else if (taskNumber == 5) {
                                intakeClaw.setPosition(0.55);
                                isOpen = false;
                            } else if (taskNumber == 6) {
                                intakeWrist.setPosition(0.84);
                            }
                            taskNumber += 1;
                            if (taskNumber > 6) {
                                alignmentTimer.cancel();
                            }
                        }
                    }
                    if (!isOpen) {
                        TimerTask alignmentTask = new alignment();
                        alignmentTimer.schedule(alignmentTask, 200, 500);
                    }
                    intakeWristTargetPos = 0.84;
                } else if (gamepad1.dpad_down/* & horizPos >= 20*/) {
                    /*downCounter += 0.3;
                    if (downCounter > 15) {
                        downCounter = 15;
                    } else if (downCounter <= 5 & downCounter >= 4) {
                        intakeArm.setPosition(0.5);
                        intakeClaw.setPosition(0.8);
                        isOpen = true;
                    }
                    if (downCounter <= 15 & downCounter >= 14) {
                        intakeArm.setPosition(0.64);
                    }*/
                    intakeArm.setPosition(0.64);
                    freeWristRotate = true;
                }
                if (vertPos > 3000 && gamepad1.right_stick_y < 0.01) {
                    verticalSlideMotor.setPower(0.3);
                } else if (gamepad1.left_bumper) {
                    verticalSlideMotor.setPower(-0.45 * gamepad1.right_stick_y);
                } else {
                    verticalSlideMotor.setPower(-gamepad1.right_stick_y);
                }
                if (gamepad1.right_stick_y > 0) {
                    verticalSlideMotor.setPower(verticalSlideMotor.getPower() * 0.5);
                }
                /*if (gamepad1.right_trigger > 0.5) {
                    verticalSlideMotor.setPower(-gamepad1.right_stick_y);
                }*/
                if (freeWristRotate) {
                    if (gamepad1.left_trigger > 0.1) {
                        intakeWristTargetPos += 0.01 * gamepad1.left_trigger;
                    } else if (gamepad1.right_trigger > 0.1) {
                        intakeWristTargetPos -= 0.005 * gamepad1.right_trigger;
                    }
                    if (intakeWristTargetPos > 0.9) {
                        intakeWristTargetPos = 0.9;
                    } else if (intakeWristTargetPos < 0.3) {
                        intakeWristTargetPos = 0.3;
                    }
                    intakeWrist.setPosition(intakeWristTargetPos);
                }

                horizPos = horizontalSlideMotor.getCurrentPosition();
                horizontalSlideMotor.setPower(-0.3 * gamepad1.left_stick_y);
                if (gamepad1.left_bumper) {
                    horizontalSlideMotor.setPower(horizontalSlideMotor.getPower() * (10 / 3));
                }
                vertPos = verticalSlideMotor.getCurrentPosition();
                telemetry.addData("vertSlidePos", vertPos);
                telemetry.addData("CurrentServoPos", targetPos);
                telemetry.addData("intakeWristRotation", intakeWristTargetPos);
                telemetry.addData("horizontalSlidePos", horizPos);
                telemetry.addData("horizontalSlidePower", horizontalSlideMotor.getPower());
                telemetry.addData("LeftStickY",gamepad1.left_stick_y);
                telemetry.addData("downCounter", downCounter);
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */

    private void moveVariables() {
        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
            vertical = -gamepad2.left_stick_y;
        } else {
            vertical = 0;
        }
        if (gamepad2.left_stick_x > 0.1 || gamepad2.left_stick_x < -0.1) {
            horizontal = gamepad2.left_stick_x;
        } else {
            horizontal = 0;
        }
        pivot = -gamepad2.right_stick_x;
    }

    private void mecanumMath() {
        rightFront.setPower(-(pivot * 0.5) - (vertical * 0.5 - horizontal * 0.5));
        rightBack.setPower(-(pivot * 0.5) - (vertical * 0.5 + horizontal * 0.5));
        leftFront.setPower(pivot * 0.5 - (vertical * 0.5 + horizontal * 0.5));
        leftBack.setPower(pivot * 0.5 - (vertical * 0.5 - horizontal * 0.5));
    }

    private void initMotorModes() {
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Describe this function...
     */
    private void speedSettings() {
        if (gamepad2.right_bumper) {
            horizontal = horizontal * 2;
            vertical = vertical * 2;
            pivot = pivot * 2;
        }
        if (gamepad2.left_bumper) {
            horizontal = (float) (horizontal * 0.5);
            vertical = (float) (vertical * 0.5);
            pivot = (float) (pivot * 0.5);
        }
    }

    private void initServoModes() {
        intakeClaw.setPosition(0.55);
        intakeWristTargetPos = 0.84;
        intakeWrist.setPosition(intakeWristTargetPos);
        intakeArm.setPosition(0);
        deliveryWrist.setPosition(0.8);
        deliveryClawR.setPosition(0.5);
        deliveryClawL.setPosition(0.3);

        intakeWristTargetPos = 0.84;
    }
}