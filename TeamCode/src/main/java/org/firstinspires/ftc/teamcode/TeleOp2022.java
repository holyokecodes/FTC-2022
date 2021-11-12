/*
Copyright 2020 FIRST Tech Challenge Team 14853
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,http://192.168.49.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/MechanumDrive.java
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Comparator;
import java.util.stream.Stream;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name = "Tele-op 2022 ")
public class TeleOp2022 extends LinearOpMode {
//    private Blinker expansion_Hub_2;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

//    private Encoder leftEncoder;
//    private Encoder rightEncoder;
//    private Encoder frontEncoder;

    //private Servo finger;
    //private DcMotorEx shooter;
    //private DcMotor intake;

    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private final double sensitivity = 1;

    static final double ServoIncrement   =  0.075; // amount to slew servo each CYCLE_MS cycle
    // Maybe make this higher
    static final int    CycleMS    = 50;     // period of each cycle
    static final double MAX_POS     =  0.6;   // Maximum rotational position
    static final double MIN_POS     =  0.3;   // Minimum rotational position
    static final double MOTOR_INCREMENT   = 0.04;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    double  power   = 0;
    boolean rampUp  = true;

    // Define class members
    double  position = 0.3; // Starting position

    float joystickDeadzone = 0.1f;

    boolean shooterOn = false;
    boolean shooterButton = false;
    boolean shooterButtonBefore = false;

    boolean intakeOn = false;
    boolean intakeButton = false;
    boolean intakeButtonBefore = false;

    int fingerState = 1;

    boolean fingerButton;
    boolean fingerButtonBefore;

    //    private double intakePower;
    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));


        // Wait for the start button
        telemetry.addData(">", "Press Start to have a call to adventure.");
        telemetry.update();

        //gamepad1.setJoystickDeadzone(joystickDeadzone); It doesn't exist in this version

        waitForStart();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double speedMultiplier = 1; //Multiplier for precision mode.
            if (gamepad1.right_trigger > 0.5) {
                speedMultiplier = 0.5;
                telemetry.addData("Pesise Mode", "On");
            } else {
                telemetry.addData("Pesise Mode", "Off");
            }


            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            double robotPower = Math.hypot(leftX, leftY);
            double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(225);

//            telemetry.addData("Joystick Angle", Math.toDegrees(robotAngle));
//            telemetry.addData("Angles (XYZ)", Math.toDegrees(angles.thirdAngle) + ", " + Math.toDegrees(angles.secondAngle) + ", " + Math.toDegrees(angles.firstAngle));

//            telemetry.addData("Front Encoder", frontEncoder.getCurrentPosition());
//            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
//            telemetry.addData("Right Encoder", frontEncoder.getCurrentPosition());


            double frontLeftPower = robotPower * Math.cos(robotAngle) + rightX;
            double frontRightPower = robotPower * Math.sin(robotAngle) - rightX;
            double backLeftPower = robotPower * Math.sin(robotAngle) + rightX;
            double backRightPower = robotPower * Math.cos(robotAngle) - rightX;

            backLeft.setPower(-backLeftPower * speedMultiplier);
            backRight.setPower(backRightPower * speedMultiplier);
            frontLeft.setPower(-frontLeftPower * speedMultiplier);
            frontRight.setPower(frontRightPower * speedMultiplier);

            /* slew the servo, according to the rampUp (direction) variable
            if (fingerState == 1) {
                Keep stepping up until we hit the max value.
                position += ServoIncrement;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    fingerState = 2;
                }
            }
            else if (fingerState == 2) {

                // Keep stepping down until we hit the min value.
                position -= ServoIncrement;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    fingerState = 0;
                }
            }

            if (fingerState == 0) telemetry.addData("Finger State", "Idle");
            else if (fingerState == 1) telemetry.addData("Finger State","Extending...");
            else if (fingerState == 2) telemetry.addData("Finger State","Retracting...");
            else telemetry.addData("Finger State","[EERRRROORR]");

            power += MOTOR_INCREMENT ;
            if (power <= MAX_REV ) {
                power = MAX_REV;
                rampUp = !rampUp;  // Switch ramp direction
            }
            // if you hit the x button, invert the state of the shooter.
            /*if (gamepad2.x){
                shooterOn = !shooterOn;
            }

            if (gamepad1.left_trigger < 0.5f){
                shooterOn = !shooterOn;
            }*/
            // Display the current values
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData("Servo Position", "%5.2f", position);

            /* Set the servo to the new position and pause;
            finger.setPosition(position);

            if (shooterOn){
                shooter.setVelocity(6600);
            } else {
                shooter.setVelocity(0);
            }
            if (intakeOn){
                intake.setPower(-power);
            } else {
                intake.setPower(0);
            }
            sleep(CycleMS);
            idle();

            telemetry.addData("Status", "Running");
            // telemetry.addData("angle", angles.firstAngle);*/
            telemetry.update();
        }

    }
}