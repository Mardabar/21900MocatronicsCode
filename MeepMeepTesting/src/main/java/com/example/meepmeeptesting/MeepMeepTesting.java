package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                /* settin stuff for redleft auto, REMEMBER TO IMPLEMENT TURNS
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-30, -61, 0))
                        .strafeTo(new Vector2d(-45,-58)) */

                // redright auto as of rn should get 3 specimin
                .setConstraints(80, 80, Math.toRadians(200), Math.toRadians(200), 13.5) //3, -62
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5, -68, 0))
                        //  .splineTo(new Vector2d(50.0, -55.0), 0.0)
                        .strafeTo(new Vector2d(5,-25))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(5, -30))
                        .turn(Math.toRadians(-180))
                        .strafeTo(new Vector2d(44,-30))
                        .strafeTo(new Vector2d(44, 1))
                        .strafeTo(new Vector2d(48,-62 ))
                        .strafeTo(new Vector2d(47,1))
                        .strafeTo(new Vector2d(57,-4.5))
                        .strafeTo(new Vector2d())
                     //   .strafeTo(new Vector2d(42, -12))
                      /*  .turn(Math.toRadians(-90))
                        .strafeTo(new Vector2d(50,-58))
                        .strafeTo(new Vector2d(47,-12))
                        .strafeTo(new Vector2d(55,-12))
                        .strafeTo(new Vector2d(55,-60))
                        .waitSeconds(.5)
                        .turn(Math.toRadians(-180))
                        .strafeTo(new Vector2d(3, -30))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(40,-56))
                        .turn(Math.toRadians(180))
                        .strafeTo(new Vector2d(50,-56))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(3,-30)) */
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}