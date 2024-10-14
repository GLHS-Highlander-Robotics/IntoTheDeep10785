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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.drive.OldDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionNew;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.constants.AutoMods;

@Disabled
public class DisabledRedFarBoard extends LinearOpMode{

    private SpikeDetectionNew spikeDetect;
    private VisionPortal portal;
    OldDrive drive;
    LinearSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {

        AutoMods.teamRed = true;
        AutoMods.isFar = true;
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
        ElapsedTime timer = new ElapsedTime();
        int forwardDist = 0;
        int rotDist = 0;
        int rightDist = 50;
        drive.imu.resetYaw();
        SpikeDetectionNew.Position position = spikeDetect.getPos();
        portal.close();
        switch (position) {
            case LEFT:
                slide.turnFloor();
                drive.rotateAndMoveInches(90, 32, 4, 0.5, 0.2);
                slide.ungrabR();
                timer.reset();
                while (timer.time() < 1) {idle();}
                slide.setAutoPos(0, LinearSlide.MEDIUM_ROT);
                forwardDist = 13;
                rotDist = LinearSlide.LOW_ROT - 10;
                break;
            case RIGHT:
                slide.turnFloor();
                drive.rotateAndMoveInches(0, 6, 29, 0.5, 0.2);
                drive.rotateAndMoveInches(0, 23, 0, 0.5, 0.2);
                drive.rotateAndMoveInches(90, 0, -8, 0.5, 0.5);
                slide.ungrabR();
                timer.reset();
                while (timer.time() < 1) {idle();}
                drive.rotateAndMoveInches(90, 0, 24, 0.5, 0.2);
                slide.setAutoPos(0, LinearSlide.MEDIUM_ROT);

                forwardDist = -5;
                rotDist = LinearSlide.LOW_ROT + 30;
                rightDist = 35;
                break;
            case CENTER:
                slide.turnFloor();
                drive.rotateAndMoveInches(180, 44, 0, 0.5, 0.75);
                slide.ungrabR();
                timer.reset();
                while (timer.time() < 1) {idle();}
                drive.rotateAndMoveInches(-90, 2, 0, 0.5, 0.75);
                drive.rotateAndMoveInches(-90, 0,40, 0.5, 0.2);
                slide.turnPlace();
                slide.setRot(LinearSlide.LOW_ROT);
                drive.rotateAndMoveInches(90, 0,0, 0.5, 0.75);
                drive.rotateAndMoveInches(90, -20, 40, 0.3, 0.2);
                forwardDist = 5;
                rotDist = LinearSlide.LOW_ROT;
                break;
            default:
                break;
        }

        drive.rotateAndMoveInches(90, 0, rightDist, 0.5, 0.5);
        slide.setAutoPos(0, rotDist);
        slide.turnPlaceEx();
        drive.rotateAndMoveInches(90, forwardDist, 20, 0.3, 0.5);
        slide.ungrabL();
        timer.reset();
        while (timer.time() < 1) {idle();}
        drive.rotateAndMoveInches(90, 0, -10, 0.3, 0.5);
        slide.setAutoPos(0,0);



    }
}
