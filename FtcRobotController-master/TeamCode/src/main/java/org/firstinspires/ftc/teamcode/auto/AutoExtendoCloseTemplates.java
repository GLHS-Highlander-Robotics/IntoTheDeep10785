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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.drive.OldDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionNew;

@Disabled
public class AutoExtendoCloseTemplates extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

    }
    LinearSlide slide;
    OldDrive drive;
    double multiplier = 1;
    SpikeDetectionNew.Position pos = SpikeDetectionNew.Position.LEFT;
    boolean isRedSide;

    public AutoExtendoCloseTemplates(LinearSlide slid, OldDrive driv, boolean redSide){
        slide = slid;
        drive=driv;
        isRedSide=redSide;
    }


    public void ungrabPurple(){
        if(isRedSide){
            slide.ungrabL();
        }else{
            slide.ungrabR();
        }
    }

    public void ungrabYellow(){
        if(isRedSide){
            slide.ungrabR();
        }else{
            slide.ungrabL();
        }
    }

    public void purplePixel(SpikeDetectionNew.Position position){
        //Initializing Constants and Inverter
        if(!isRedSide){
            multiplier = -1;
            if(position==SpikeDetectionNew.Position.LEFT){
                pos=SpikeDetectionNew.Position.RIGHT;
            }else if(position==SpikeDetectionNew.Position.RIGHT){
                pos=SpikeDetectionNew.Position.LEFT;
            }else{
                pos=SpikeDetectionNew.Position.CENTER;
            }
        } else {
            multiplier = 1;
            pos=position;
        };

        //Actual Code
        drive.rotateAndMoveInches(0,10,0,0.5,0.5);
        switch (pos){
            //Remember that this is on Red side

            case RIGHT:
                //Close to Wall/Corner
                drive.rotateAndMoveInches(-25*multiplier, 0,0,0,0.4);
                slide.setAutoExtendo((int)(18/slide.INCHESPERTICK));
                slide.turnFloorEx();
                wait(0.25);
                ungrabPurple();
                wait(0.25);
                slide.turnPlaceEx();
                slide.setAutoExtendo(0);
                wait(0.5);
                drive.rotateAndMoveInches(0,0,0,0,0.5);
                drive.rotateAndMoveInches(0,-2.5,0,0.5,0.75);
                break;
            case CENTER:
                slide.setAutoExtendo((int)(22.0/slide.INCHESPERTICK));
                slide.turnFloorEx();
                wait(0.25);
                ungrabPurple();
                wait(0.25);
                slide.turnPlaceEx();
                slide.setAutoExtendo(0);
                wait(0.5);
                drive.rotateAndMoveInches(0,0,0,0,0.5);
                drive.rotateAndMoveInches(0,-2,0,0.5,0.75);
                break;
            case LEFT:
                //Far from Wall/Corner
                drive.rotateAndMoveInches(20*multiplier, 0,0,0,0.4);
                slide.setAutoExtendo((int)(18/slide.INCHESPERTICK));
                slide.turnFloorEx();
                wait(0.25);
                ungrabPurple();
                wait(0.25);
                slide.turnPlaceEx();
                slide.setAutoExtendo(0);
                wait(0.5);
                drive.rotateAndMoveInches(0,0,0,0,0.5);
                drive.rotateAndMoveInches(0,-3.5,0,0.5,0.75);
                break;
        }
    }

    public void yellowPixelIn(SpikeDetectionNew.Position position){
        //Initializing Constants and Inverter
        if(!isRedSide){
            multiplier = -1;
            if(position==SpikeDetectionNew.Position.LEFT){
                pos=SpikeDetectionNew.Position.RIGHT;
            }else if(position==SpikeDetectionNew.Position.RIGHT){
                pos=SpikeDetectionNew.Position.LEFT;
            }else{
                pos=SpikeDetectionNew.Position.CENTER;
            }
        } else {
            multiplier = 1;
            pos=position;
        };

        //Actual Code
        drive.rotateAndMoveInches(0,0,39*multiplier,0.5,0.5);
        switch(pos){
            //Remember that this is on Red side

            case LEFT:
                //Far from Wall/Corner
                drive.rotateAndMoveInches(90*multiplier,30.5,0,0.5,0.25);
                slide.setArmPos(350, LinearSlide.LOW_ROT-10);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90*multiplier,0,16*multiplier,0.25,0.2);
                wait(1.0);
                break;

            case RIGHT:
                //Close to Wall/Corner
                drive.rotateAndMoveInches(90*multiplier,17.5,0,0.5,0.25);
                slide.setArmPos(350, LinearSlide.LOW_ROT-10);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90*multiplier,0,16*multiplier,0.25,0.2);
                wait(1.0);
                break;

            case CENTER:
                drive.rotateAndMoveInches(90*multiplier,22,0,0.5,0.25);
                slide.setArmPos(300, LinearSlide.LOW_ROT-5);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90*multiplier,0,17*multiplier,0.25,0.2);
                wait(0.75);
                break;
        }
        ungrabYellow();
        wait(0.5);
        slide.setAutoPos(0,0);
    }

    public void parkCenter(SpikeDetectionNew.Position position){
        //Initializing Constants and Inverter
        if(!isRedSide){
            multiplier = -1;
            if(position==SpikeDetectionNew.Position.LEFT){
                pos=SpikeDetectionNew.Position.RIGHT;
            }else if(position==SpikeDetectionNew.Position.RIGHT){
                pos=SpikeDetectionNew.Position.LEFT;
            }else{
                pos=SpikeDetectionNew.Position.CENTER;
            }
        } else {
            multiplier = 1;
            pos=position;
        };

        //Actual Code
        switch(pos){
            //Remember that this is on Red side
            case LEFT:
                //Far from Wall/Corner
                drive.rotateAndMoveInches(90*multiplier,0,-10*multiplier,0.4,0.2);
                drive.rotateAndMoveInches(90*multiplier,20,0,0.4,0.2);
//                drive.rotateAndMoveInches(0,0,17,0.4,0.4);
                break;
            case CENTER:
                drive.rotateAndMoveInches(90*multiplier,27,-10*multiplier,0.4,0.2);
//                drive.rotateAndMoveInches(0,0,17,0.4,0.4);
                break;
            case RIGHT:
                //Close to Wall/Corner
                drive.rotateAndMoveInches(90*multiplier,36,-10*multiplier,0.4,0.2);
//                drive.rotateAndMoveInches(0,0,17,0.4,0.4);
                break;
        }
        drive.rotateAndMoveInches(0,0,17*multiplier,0.4,0.4);
        slide.setAutoPos(0,0);
    }

    public void wait (double t) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time() < t) {idle();}
    }
}
