/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.AutoMods;
import org.firstinspires.ftc.teamcode.subsystem.drive.OldDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionNew;
import org.firstinspires.ftc.vision.VisionPortal;
@Disabled
@Autonomous(name = "RedBoard")
public class DisabledAutoRedBoard extends LinearOpMode{

    private SpikeDetectionNew spikeDetect;
    private VisionPortal portal;
    OldDrive drive;
    LinearSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoMods.teamRed = true;
        AutoMods.isFar = false;
        slide = new LinearSlide(hardwareMap);
        drive = new OldDrive(hardwareMap, this);
        drive.botHeading = 0;


        spikeDetect = new SpikeDetectionNew();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(spikeDetect)
                .build();

        slide.turnPlaceAuto();
        slide.grabAll();
        slide.turnRot(slide.droneServo, 1);


        waitForStart();
        drive.imu.resetYaw();
        SpikeDetectionNew.Position position = spikeDetect.getPos();
        portal.close();
        switch (position) {
            case LEFT:
                //Place First Purple Pixel
                slide.turnFloor();
                drive.rotateAndMoveInches(90, 30, 5, 0.5, 0.2);
                slide.setArmPos(150,0);
                drive.rotateAndMoveInches(90, 0, -2, 0.25, 0.25);
                drive.rotateAndMoveInches(90, 0, 2, 0.25, 0.25);
                slide.ungrabR();
                wait(0.25);
                //Move to Wall to place Yellow Pixel
                slide.setArmPos(0, LinearSlide.LOW_ROT + 25);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90,0,-5,0.4,0.25);
                wait(0.3);
                drive.rotateAndMoveInches(90, 14, 45, 0.5, 0.25);
                wait(0.25);
                slide.ungrabL();
                wait(0.25);
                drive.rotateAndMoveInches(90, 21, -15, 0.5, 0.25);

                break;
            case RIGHT:
                slide.turnFloor();
                drive.rotateAndMoveInches(90, 30, 31, 0.5, 0.2);
                slide.setArmPos(150,0);
                drive.rotateAndMoveInches(90, 0, -2, 0.25, 0.25);
                drive.rotateAndMoveInches(90, 0, 2, 0.25, 0.25);
                slide.ungrabR();
                wait(0.25);
                //Move to Wall to place Yellow Pixel
                slide.setArmPos(0, LinearSlide.LOW_ROT + 25);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90, -3, 27, 0.5, 0.5);
                wait(0.75);
                slide.ungrabL();
                wait(0.35);
                drive.rotateAndMoveInches(90, 0, -13, 0.5, 0.5);
                drive.rotateAndMoveInches(90, 31.5, -0, 0.5, 0.5);

                break;
            case CENTER:
                slide.turnFloor();
                drive.rotateAndMoveInches(0, 30, 0, 0.5, 0.2);
                slide.setArmPos(150,0);
                drive.rotateAndMoveInches(90, 2, 0, 0.25, 0.25);
                drive.rotateAndMoveInches(90, -2, 0, 0.25, 0.25);
                slide.ungrabR();
                wait(0.4);
                slide.setArmPos(0, LinearSlide.LOW_ROT +10);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90, 3, 50, 0.5, 0.5);
                slide.ungrabL();
                wait(0.4);
                drive.rotateAndMoveInches(90, 0, -13, 0.5, 0.5);
                slide.setArmPos(0, LinearSlide.MEDIUM_ROT + 20);
                drive.rotateAndMoveInches(90, 27, 0, 0.5, 0.5);

                break;
            default:
                break;
        }
        //Discrepancy Line 154, 142
        //Go to get 2 White pixels
        //Go Under Gate
        slide.setArmPos(0, LinearSlide.MEDIUM_ROT + 20);
        slide.turnFloor();
        drive.rotateAndMoveInches(90, 0, -60, 0.6, 0.25);
        slide.setArmPos(100,0);
        drive.rotateAndMoveInches(90, 0.5, -31, 0.5, 0.25);

        //Pick up White Pixel
        wait(2.0);
        drive.rotateAndMoveInches(90, 0, -14.75, 0.2, 0.25);

//        drive.rotateAndMoveInches(90, 2, -14.75, 0.2, 0.25);
        wait(0.5);
        slide.grabL();
        slide.grabR();

        wait(0.5);
        //Go Back to Wall
        drive.rotateAndMoveInches(90, 0, 8.5, 0.7, 0.25);
        slide.setArmPos(0, LinearSlide.MEDIUM_ROT+20);
        slide.turnPlaceEx();
        wait(2.0);
        drive.rotateAndMoveInches(90, 0, 80, 1.0, 0.25);

        drive.rotateAndMoveInches(90, -46, 15, 0.95, 0.25);
        wait(0.5);
        //Move to Wall to place White Pixel

        slide.setArmPos(200, LinearSlide.LOW_ROT - 20);
        wait(1.0);
//        slide.turnPlaceEx();
//        wait(0.5);
        drive.rotateAndMoveInches(90, 0, 35, 0.5, 0.25);
        drive.rotateAndMoveInches(90, 0, 2, 0.5, 0.25);
        slide.ungrabL();
        wait(0.5);
        slide.setAutoPos(0,0);
        drive.rotateAndMoveInches(90, 0, -7, 0.3, 0.25);
        slide.setAutoPos(0,0);

    }

    public void wait (double t) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time() < t) {idle();}
    }
}
