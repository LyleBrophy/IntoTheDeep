package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "crying")
public class hangTest extends LinearOpMode {
    public DcMotor motorFL, motorFR, motorBR, motorBL;

    public DcMotor leftTubeDrive, rightTubeDrive;
    public DcMotor hangOne;

    public DcMotor[] drivetrain = new DcMotor[4];
    DcMotor body;
    DcMotor wrist;
    CRServo rightClaw, leftClaw;
    CRServo grabbie;

    public double speedMode = 0.7;

    public double lilUpWristSpeedMode = 0.6;

    public double lilDownWristSpeedMode = -.5;


    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        body = hardwareMap.get(DcMotor.class, "body");
        leftTubeDrive = hardwareMap.get(DcMotor.class, "leftTubeDrive");
        rightTubeDrive = hardwareMap.get(DcMotor.class, "rightTubeDrive");
        hangOne = hardwareMap.get(DcMotor.class, "hangOne");
//        arm = hardwareMap.get(DcMotor.class, "arm");
//        wrist = hardwareMap.get(DcMotor.class, "wrist");

        grabbie = hardwareMap.get(CRServo.class, "grabbie");
        rightClaw = hardwareMap.get(CRServo.class, "rightClaw");
        leftClaw = hardwareMap.get(CRServo.class, "leftClaw");

        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        rightClaw.setPower(0);
        leftClaw.setPower(0);

        body.setDirection(DcMotorSimple.Direction.FORWARD);
        leftTubeDrive.setPower(0);
        rightTubeDrive.setPower(0);
        body.setPower(0);
        hangOne.setPower(0);
        hangOne.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTubeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTubeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        body.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                speedMode = 1;
            } else if (gamepad1.right_bumper) {
                speedMode = 0.2;
            } else if (gamepad1.a) {
                speedMode = 0.7;
            } else {
                speedMode = 0.7;
            }
            double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
            double strafe = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
            double turn = -speedMode * Math.pow(gamepad1.right_stick_x, 3);
            double frontLeftPower = forward + strafe + turn;
            double backLeftPower = forward - strafe + turn;
            double frontRightPower = forward - strafe - turn;
            double backRightPower = forward + strafe - turn;
            double[] powers = {frontLeftPower, backLeftPower, frontRightPower, backRightPower};
            boolean needToScale = false;
            for (double power : powers) {
                if (Math.abs(power) > 1) {
                    needToScale = true;
                    break;
                }
            }
            if (needToScale) {
                double greatest = 0;
                for (double power : powers) {
                    if (Math.abs(power) > greatest) {
                        greatest = Math.abs(power);
                    }
                }
                frontLeftPower /= greatest;
                backLeftPower /= greatest;
                frontRightPower /= greatest;
                backRightPower /= greatest;
            }
            boolean stop = true;
            for (double power : powers) {
                double stopBuffer = 0;
                if (Math.abs(power) > stopBuffer) {
                    stop = false;
                    break;
                }
            }

            if (stop) {
                frontLeftPower = 0;
                backLeftPower = 0;
                frontRightPower = 0;
                backRightPower = 0;
            }
            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            if (gamepad2.left_stick_y < -0.1) {
                body.setPower(-0.7);
            } else if (gamepad2.left_stick_y > 0.1) {
                body.setPower(0.7);
            } else if (gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1) {
                body.setPower(0);
            }

            if (gamepad1.dpad_up) {
                hangOne.setPower(1);
            } else if (gamepad1.dpad_down){
                hangOne.setPower(-1);
            } else if (gamepad1.dpad_up == false && gamepad1.dpad_down == false) {
                hangOne.setPower(0);
            }



//basmalah is both the best and the worst at the same time

            if (gamepad2.dpad_up) {
                leftTubeDrive.setPower(-1);
                rightTubeDrive.setPower(1);
            } else if (gamepad2.dpad_down) {
                leftTubeDrive.setPower(1);
                rightTubeDrive.setPower(-1);
            } else if (gamepad2.dpad_up == false && gamepad2.dpad_down == false) {
                leftTubeDrive.setPower(0);
                rightTubeDrive.setPower(0);
            }

            if (gamepad2.x) {
                grabbie.setPower(-1);
//                leftClaw.setPower(-1);
//                rightClaw.setPower(1);
            } else if (gamepad2.y) {
                grabbie.setPower(1);
//                leftClaw.setPower(1);
//                rightClaw.setPower(-1);
            } else if (gamepad2.x == false && gamepad2.y == false) {
                grabbie.setPower(0);
//                leftClaw.setPower(0);
//                rightClaw.setPower(0);
            }


            }
        }
    }


