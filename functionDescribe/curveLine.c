/*
 *곡선(curveLine) 
 *  parameter : resHeight, resWidth, imgResult, last_width
 *  global prameter : corner_check, last_height
 *  define parameter : curve_flag(int), j, k, curve_temp, center_avg, sub_center, steer_angle
 */
void curveLine(int resHeight, int resWidth, IplImage* imgResult, int last_width)
{
    int curve_flag, curve_temp, sub_center, steer_angle;
    int j, k;

    curve_flag=0;

    printf("Corner in!!!!!!!!!!!!!!!!!!!\n");

    for(j = resHeight-1; j >= 0; j--)
    {
        for(k = resWidth-1; k >= 0; k--)
        {
            if(imgResult->imageData[j*imgResult->widthStep + k] == (char) 255)
            {
                last_height = j;
                //last_width = k;
                curve_flag++;
                break;
            }
        }
        if(curve_flag!=0)
        {
            curve_flag = 0;
            break;
        }
    }   //영상 제일 작은 Height값 검출

    for(j = resHeight-1; j >= 0; j--)
    {
        for(k = resWidth-1; k >= 0; k--)
        {
            if(tempResult->imageData[j*tempResult->widthStep + k] == (char) 255)
            {
                //last_height = j;
                last_width = k;
                curve_flag++;
                break;
            } //좌 또는 우에서 최초로 나타나는 점 pixel 확인
        }
        if(curve_flag!=0)
        {
            curve_flag = 0;
            break;
        }
    }   //좌우 곡선을 판단하기 위한 로직

    for(j = 0; j < 20; j++)
    {
        if(distance_table[j][1] >= last_height )
        {
            if(distance_table[j+1][1] < last_height)
            {
                encoder_value = distance_table[j][0]+31;
                //Nak
                printf("Stop!!!\n");
                printf("encoder_value : %d\n", encoder_value); //몇 센치 이동하는지 check
                //~Nak
                break;
            }
        }
    } //Table에서 움직인 거리 읽어옴

    UserMove(1, 1470, encoder_value*10);
    curve_temp = 675;  //곡선 인식 후 코너 진입점 까지 주행


    // 각 코너별 flag 만들어서 진행!!!!
    if(last_width<(resWidth/2))
    {

        printf("--------------RIGHT---------------------\n");

        //				CameraXServoControl_Write(1270);
        UserMove(1, 1000, curve_temp);
        printf("---------------------------------finish!\n");

    } //오른쪽으로 회전 
    else
    {	
        printf("---------------LEFT---------------------\n");

        //				CameraXServoControl_Write(1670);
        UserMove(1, 1950, curve_temp);
        printf("---------------------------------finish!\n");

    } // 왼쪽으로 회전

    encoder_value=0;
}
