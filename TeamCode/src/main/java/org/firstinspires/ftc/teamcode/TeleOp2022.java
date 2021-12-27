/*
2020-2021 FIRST Tech Challenge Team 14853
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor carousel;
    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    public void moveServo(Servo servo, float speed, long time) {
        servo.setPosition(speed);
        long startTime = System.currentTimeMillis();
        while (true) {
            if (System.currentTimeMillis() - startTime >= time) {
                break;
            }
        }
        servo.setPosition(0.5)
    }

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");


        // Wait for the start button
        telemetry.addData(">", "Press Start to have a call to adventure.");
        telemetry.update();

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
                telemetry.addData("Precise Mode", "On");
            } else {
                telemetry.addData("Precise Mode", "Off");
            }

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            double robotPower = Math.hypot(leftX, leftY);
            double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(45);

//            telemetry.addData("Joystick Angle", Math.toDegrees(robotAngle));
//            telemetry.addData("Angles (XYZ)", Math.toDegrees(angles.thirdAngle) + ", " + Math.toDegrees(angles.secondAngle) + ", " + Math.toDegrees(angles.firstAngle))

            double frontLeftPower = robotPower * Math.cos(robotAngle) + rightX;
            double frontRightPower = robotPower * Math.sin(robotAngle) - rightX;
            double backLeftPower = robotPower * Math.sin(robotAngle) + rightX;
            double backRightPower = robotPower * Math.cos(robotAngle) - rightX;

            backLeft.setPower(-backLeftPower * speedMultiplier);
            backRight.setPower(backRightPower * speedMultiplier);
            frontLeft.setPower(-frontLeftPower * speedMultiplier);
            frontRight.setPower(frontRightPower * speedMultiplier);

            if (gamepad1.a) {
                carousel.setPower(-.1);
            } else {
                carousel.setPower(0);
            }
            /*
            * if(gamepad1.a){
            *   running = !running;
            * }
            * carousel.setPower(running);
            * */
            telemetry.update();
        }
    }
}