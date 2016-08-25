        /*
         * 주행(driveLine)
         * global parameter : center_avg, center_total, center_num
         * define parameter : sub_center, steer_angle
         */
void driveLine()
{
    int sub_center, steer_angle; 

    printf("Go straight!!!!!!!!!!!!!!!!!\n");
    CameraXServoControl_Write(1470);

    center_avg = center_total / center_num; //Frame2Ipl에서 측정한 중간 값의 평균을 계산
    sub_center = 160 - center_avg;          //영상의 중간과의 차이 계산
    steer_angle += 4*sub_center;            //영상의 중앙으로 가기 위해 조향각 변화
    SteeringServoControl_Write(steer_angle);
}
