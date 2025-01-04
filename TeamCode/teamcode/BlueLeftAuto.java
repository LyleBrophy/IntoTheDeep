package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous (name = "BlueLeftAuto")

public class BlueLeftAuto extends LinearOpMode {
    public DcMotor[] drivetrain = new DcMotor[4];
    public DcMotor motorFL, motorBL, motorBR, motorFR,leftTubeDrive, rightTubeDrive, body, hangOne;
    CRServo grabbie;
    private BNO055IMU imu = null;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;


    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int frontleftTarget = 0;
    private int frontrightTarget = 0;
    private int backleftTarget = 0;
    private int backrightTarget = 0;
    Integer cpr = 570;
    Integer gearRatio = 1;
    Double diameter = 4.125;
    Double cpi = (cpr * gearRatio) / (Math.PI * diameter);
    Double meccyBias = 0.9;
    Double bias = 0.8;
    Double conversions = cpi * bias;
    Boolean exit = false;
    Integer COUNTS_PER_MOTOR_REV = 570;
    Integer DRIVE_GEAR_REDUCTION = 1;
    Double WHEEL_DIAMETER_INCHES = 4.125;
    Double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER_INCHES); //counts per inch
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.2;
    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;
    static final double P_DRIVE_COEFF = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system variables.
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        leftTubeDrive = hardwareMap.get(DcMotor.class, "leftTubeDrive");
        rightTubeDrive = hardwareMap.get(DcMotor.class, "rightTubeDrive");
        body = hardwareMap.get(DcMotor.class, "body");
        hangOne = hardwareMap.get(DcMotor.class, "hangOne");

        grabbie = hardwareMap.get(CRServo.class, "grabbie");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        leftTubeDrive.setDirection(DcMotor.Direction.FORWARD);
        rightTubeDrive.setDirection(DcMotor.Direction.FORWARD);
        body.setDirection(DcMotor.Direction.FORWARD);
        hangOne.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        leftTubeDrive.setPower(0);
        rightTubeDrive.setPower(0);
        body.setPower(0);
        hangOne.setPower(0);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftTubeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTubeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        body.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTubeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTubeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        body.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();
        telemetry.addLine("I'm just a girl");
        telemetry.addLine("Lyle est tres tres yuck☹");
        while (opModeInInit()) {
            telemetry.update();
        }

        waitForStart();
        Strafe(0.7,250);
//        grabbie.setPower(1);
//        tubeDrive(1,17);
//        moveBody(-0.7,15);
//        Strafe(-0.5,5);
//        IMUDrive(0.7, 40,0);
//        Strafe(-0.5,5);
//        moveBody(0.7, 5);
//        tubeDrive(1, 5);
//        openGrabbie();
//        IMUHold(0.2,0,500);
//        IMUDrive(0.5,-5,0);
//        Strafe(-0.5,10);
//        IMUHold(0.2,0,500);
//        tubeDrive(-1,3);
//        openGrabbie();
//        moveBody(0.7,10);
//        closeGrabbie();
//        moveBody(-0.7,10);
//        IMUDrive(-0.7,5,0);
//        IMUHold(0.2,                                                                                                                                                                    0,500);
//        IMUTurn(0.7,180);
//        IMUHold(0.2,180,500);
//        motorFL.setPower(1);
//        motorBL.setPower(1);
//        motorBR.setPower(1);
//        motorFR.setPower(1);
//        sleep(50000);

    }

    public void IMUDrive(double maxDriveSpeed,
                         double distance,
                         double heading) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            frontleftTarget = motorFL.getCurrentPosition() + moveCounts;
            frontrightTarget = motorFR.getCurrentPosition() + moveCounts;
            backleftTarget = motorBL.getCurrentPosition() + moveCounts;
            backrightTarget = motorBR.getCurrentPosition() + moveCounts;
            // Set Target FIRST, then turn on RUN_TO_POSITION
            motorFL.setTargetPosition(frontleftTarget);
            motorFR.setTargetPosition(frontrightTarget);
            motorBL.setTargetPosition(backleftTarget);
            motorBR.setTargetPosition(backrightTarget);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy())) {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;
                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);
                // Display drive status for the driver.
                sendTelemetry(true);
            }
            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void StrafyIMU(double speed, double inches, double angle) {
        int move = (int) (Math.round(inches * cpi * meccyBias));
        int newFrontLeftTarget = motorFL.getCurrentPosition() + move;
        int newFrontRightTarget = motorFR.getCurrentPosition() - move;
        int newBackLeftTarget = motorBL.getCurrentPosition() - move;
        int newBackRightTarget = motorBR.getCurrentPosition() + move;


        motorBL.setTargetPosition(newBackLeftTarget);
        motorFL.setTargetPosition(newFrontLeftTarget);
        motorBR.setTargetPosition(newBackRightTarget);
        motorFR.setTargetPosition(newFrontRightTarget);


        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorFL.setPower(speed);
        motorBL.setPower(speed);
        motorFR.setPower(speed);
        motorBR.setPower(speed);


        while ((motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy())) {
            double error = getError(angle);
            double steer = getSteer(error, P_DRIVE_COEFF);


            if (inches < 0) steer *= -1.0;


            double frontLeftSpeed = speed - steer;
            double backLeftSpeed = speed + steer;
            double backRightSpeed = speed + steer;
            double frontRightSpeed = speed - steer;


            double max = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed)), Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed)));
            if (max > 1.0) {
                frontLeftSpeed /= max;
                frontRightSpeed /= max;
                backLeftSpeed /= max;
                backRightSpeed /= max;
            }


            motorFL.setPower(frontLeftSpeed);
            motorFR.setPower(frontRightSpeed);
            motorBL.setPower(backLeftSpeed);
            motorBR.setPower(backRightSpeed);


        }


        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }

    private double getError(double targetAngle) {
        double robotError = targetAngle - getCurrentHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getCurrentHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void IMUTurn(double maxTurnSpeed, double heading) {
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            // Display drive status for the driver.
            sendTelemetry(false);
        }
        // Stop all motion;
        moveRobot(0, 0);
    }

    public void IMUHold(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            // Display drive status for the driver.
            sendTelemetry(false);
        }
        // Stop all motion;
        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;
        // Determine the heading current error
        headingError = targetHeading - robotHeading;
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.
        leftSpeed = drive - turn;
        rightSpeed = drive + turn;
        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        motorFL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed);
        motorBL.setPower(leftSpeed);
        motorBR.setPower(rightSpeed);
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", frontleftTarget, frontrightTarget, backleftTarget, backrightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", motorFL.getCurrentPosition(),
                    motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void Strafe(double power, double inches) {
        int move = (int) (Math.round(inches * conversions));
        motorFL.setTargetPosition(motorFL.getCurrentPosition() - move);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() + move);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() CKMMM                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;/////////..................













        






        //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 + move);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - move);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setPower(-power);
        motorFR.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power);
        while (motorFL.isBusy() && motorFR.isBusy()) {
            if (exit) {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFR.setPower(0);
                motorFL.setPower(0);
            }
        }
    }
    public void tubeDrive (double power, double inches) {
        int move = (int) (Math.round(inches * 15 * conversions));
        leftTubeDrive.setTargetPosition(leftTubeDrive.getCurrentPosition() - move);
        rightTubeDrive.setTargetPosition(rightTubeDrive.getCurrentPosition() + move);
        leftTubeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTubeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftTubeDrive.setPower(power);
        rightTubeDrive.setPower(power);
        while (leftTubeDrive.isBusy() && rightTubeDrive.isBusy()) {
            if (exit) {
                leftTubeDrive.setPower(0);
                rightTubeDrive.setPower(0);
            }
        }
    }
    public void moveBody(double power, double inches) {
        int move = (int) (Math.round(inches * conversions));
        body.setTargetPosition(body.getCurrentPosition() + move);
        body.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        body.setPower(power);
        while (body.isBusy()) {
            if (exit) {
                body.setPower(0);
            }
        }
    }
    public void moveHanger(double power, double inches) {
        int move = (int) (Math.round(inches * conversions));
        hangOne.setTargetPosition(body.getCurrentPosition() + move);
        hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangOne.setPower(power);
        while (hangOne.isBusy()) {
            if (exit) {
                hangOne.setPower(0);
            }
        }
    }

    public void openGrabbie() {
        grabbie.setPower(-1);
        sleep(500);
    }

    public void closeGrabbie() {
        grabbie.setPower(1);
        sleep(500);
    }
}
