package org.firstinspires.ftc.teamcode.Tank;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class ArrayBased extends LinearOpMode {


    static int[][] pos = { {0,0,0,0,0,0},
                           {0,0,0,0,0,0},
                           {0,0,0,0,0,0},
                           {0,0,1,0,0,0},
                           {0,0,0,0,0,0},
                           {0,0,0,0,0,0} };


    public static void navigate(int targetx,int targety, int currentx, int currenty){

        int difX = targetx - currentx;
        int difY = targety - currenty;

        for(int i = 0; i < difX;i++){
            moveRight();
        }
        for(int i = 0; i > difX;i--){
            moveLeft();
        }
        for(int i = 0;i < difY;i++){
            moveDown();
        }
        for(int i = 0; i > difY;i--){
            moveUp();
        }

        pos[currenty][currentx] = 0;
        pos[targety][targetx] = 1;

    }

    public static void moveLeft(){}
    public static void moveRight(){}
    public static void moveUp(){}
    public static void moveDown(){}

    public void runOpMode() throws InterruptedException{

        while(opModeIsActive()){
            navigate(0,0,returnCurrent()[1],returnCurrent()[0]);
            Thread.sleep(5000);
            navigate(4,4,returnCurrent()[1],returnCurrent()[0]);
            Thread.sleep(10000);
        }
    }

    public int[] returnCurrent(){
        for(int r = 0; r < pos.length;r++){
            for(int c = 0; c < pos[0].length;c++){
                if(pos[r][c] == 1){
                    return new int[] {r,c};
                }
            }
        }
        return new int[] {0,0};
    }


}

