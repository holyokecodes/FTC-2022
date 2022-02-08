package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


@Autonomous(name="Autonomous 2022 Red A")

public class Autonomize extends LinearOpMode {
    SampleMecanumDrive drivetrain;

    private Pose2d start = new Pose2d(0, 0, Math.toRadians(180)); //Where the robot starts

    private Pose2d shippingHub = new Pose2d(20, 20, Math.toRadians(180)); //Where the robot is after going to zone c


    Trajectory shippingHubTraj;
    Trajectory duckTraj;

    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(hardwareMap);

        shippingHubTraj = drivetrain.trajectoryBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(-12, -40), Math.toRadians(90))
                .splineTo(new Vector2d(-60, -50),
                            0,
                                       SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        duckTraj = drivetrain.trajectoryBuilder(new Pose2d(-12, -40, Math.toRadians(90)))

                .build();


        /**CReturnTrajectory = drivetrain.trajectoryBuilder(cZone)
         .strafeTo(whiteLine)
         .build();
         */


        /**     EndTrajectory = drivetrain.trajectoryBuilder(whiteLinePose)
         .strafeTo(new Vector2d(55, 0))
         .build();
         */


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode, wait for Charles III to see things though.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.

            drivetrain.followTrajectory(shippingHubTraj);
        }
    }
}