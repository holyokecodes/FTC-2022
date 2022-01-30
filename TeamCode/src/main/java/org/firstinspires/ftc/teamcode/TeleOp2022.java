
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Tele-op 2022")
public class TeleOp2022 extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor carousel;
    private DcMotor arm;
    private CRServo grabber;

    private int[] armPositions = {0, -50, -125, -225, -325};
    private int currentArmPosition = 0;
    private boolean aPressed = false;
    private boolean bPressed = false;

    /*private void moveServoPower(CRServo servo, double power, float time) {
        servo.setPower(power);
        long startTime = System.currentTimeMillis();
        while (true) {
            if (System.currentTimeMillis() - startTime >= time) {
                break;
            }
        }
        servo.setPower(0);
    }

    private void moveServo(Servo servo, double position, float time) {
        servo.setPosition(position);
        long startTime = System.currentTimeMillis();
        while (true) {
            if (System.currentTimeMillis() - startTime >= time) {
                break;
            }
        }
    }

    private void openGrabber(float time){
        moveServo(grabber, 0.35, time);
    }
    private void closeGrabber(float time){
        moveServo(grabber, 0.55, time);
    }*/
    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        // Wait for the start button
        telemetry.addData(">", "Press Start to energize the robot with electrons that make it MOVE!");
        telemetry.update();

        //initialize arm
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double speedMultiplier = 1; //Multiplier for precision mode.
            if (gamepad1.right_trigger > 0.5) {
                speedMultiplier = 0.25;
                telemetry.addData("Precise Mode", "On");
            } else {
                telemetry.addData("Precise Mode", "Off");
            }

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

            if (gamepad2.x) {
                carousel.setPower(-0.8);
            } else if (gamepad2.y) {
                carousel.setPower(0.8);
            }else{
                carousel.setPower(0);
            }

            //handles arm movement
            //telemetry.addData("Ticks", arm.getCurrentPosition());
            telemetry.addData("Current Arm Position", currentArmPosition);

            if(gamepad2.a && !aPressed){
                if(currentArmPosition<armPositions.length-1){
                    currentArmPosition++;
                }
                arm.setTargetPosition(armPositions[currentArmPosition]);
                arm.setPower(-.1);
                aPressed = true;
            }else if (gamepad2.b && !bPressed) {
                if(currentArmPosition>0){
                    currentArmPosition--;
                }
                arm.setTargetPosition(armPositions[currentArmPosition]);
                arm.setPower(.1);
                bPressed = true;
            }

            aPressed = gamepad2.a;
            bPressed = gamepad2.b;

            if(gamepad2.left_bumper){
                //openGrabber(100);
                grabber.setPower(.1);
            }else if (gamepad2.right_bumper){
                //closeGrabber(100);
                grabber.setPower(-.1);
            }else{
                grabber.setPower(0);
            }

            telemetry.addData("Current Position", arm.getCurrentPosition());
            telemetry.addData("Projected Position", armPositions[currentArmPosition]);
            telemetry.update();

            idle();
        }
    }
}