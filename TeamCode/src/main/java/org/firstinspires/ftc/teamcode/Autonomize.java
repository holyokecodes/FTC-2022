package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name="Autonomous 2022 Red A")

public class Autonomize extends LinearOpMode {
    SampleMecanumDrive drivetrain;

    private DcMotor carousel;
    private DcMotor arm;
    private CRServo grabber;

    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(hardwareMap);

        carousel = hardwareMap.get(DcMotor.class, "carousel");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drivetrain.setPoseEstimate(startPose);

        //initialize arm
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrajectorySequence trajSeq = drivetrain.trajectorySequenceBuilder(startPose)
                // Raise arm
                .addTemporalMarker(() -> arm.setPower(-.1))
                .addTemporalMarker(() -> arm.setTargetPosition(-325))
                .waitSeconds(2)
                // Goto shipping hub
                .lineTo(new Vector2d(-6,-32))
                // Drop cargo
                .addTemporalMarker(() -> grabber.setPower(-.1))
                .waitSeconds(.5)
                .addTemporalMarker(() -> grabber.setPower(0))
                // Goto carousel
                .lineTo(new Vector2d(-70,-54))
                .turn(Math.toRadians(30))
                .back(5)
                // Spin duck carousel
                .addTemporalMarker(() -> carousel.setPower(-.8))
                .waitSeconds(4)
                .addTemporalMarker(() -> carousel.setPower(0))
                .build();


        telemetry.addData(">", "Press Play to start op mode, amd wait for the movement of the robot.");
        telemetry.update();

        /** Wait for the game to begin */
        waitForStart();

        if (opModeIsActive()) {
            drivetrain.followTrajectorySequence(trajSeq);
        }
    }
}