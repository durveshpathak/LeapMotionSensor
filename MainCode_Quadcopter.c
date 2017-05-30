
#include "mbed.h"
#include "FXOS8700Q.h"
#include "FXAS21000.h"
#include "kalman.c"
#include "arm_math.h"
#include "hcsr04.h"
#include "nRF24L01P.h"

#define CONVG 3.9
#define Rad2Deg 57.295779
#define PI 3.14159
#define ROLL_ref            PI/2
#define PITCH_ref          PI/2

#define PID_ROLL_KP        0.0245f              /* Proporcional */ //0.015f
#define PID_ROLL_KI        0.000175f              /* Integral */
#define PID_ROLL_KD        0.0f              /* Derivative */

#define PID_PITCH_KP       0.0245f              /* Proporcional */ //0.015f
#define PID_PITCH_KI       0.000175f              /* Integral */
#define PID_PITCH_KD       0.0f              /* Derivative */
#define TRANSFER_SIZE      4

kalman filter_pitch; 
kalman filter_roll;
Timer GlobalTime;
Timer ProgramTimer;
double angle[3];
unsigned long timer;
long loopStartTime;

float error_ROLL;
float error_PITCH;
bool Flag = 0;
float pitch,roll;
char command = ' ';
float high;
char data;

PwmOut M1(D13);  // Electronic speed controller connections
PwmOut M2(D12);
PwmOut M3(D11);
PwmOut M4(D10);

double ESC1 = 0.0001f;   //pitch up
double ESC2 = 0.0001f;   //roll  up
double ESC3 = 0.0001f;   //roll  down
double ESC4 = 0.0001f;   //pitch down

float gyro_data[3];             //Variable definition for IMU sensor
float acc_pitch_angle,acc_roll_angle,acc_yaw_angle ;
float xaccel=0.0;
float yaccel=0.0;
float zaccel=0.0;
float R;
float xmean,ymean,zmean,xGmean,yGmean,zGmean;
float x_acc_offset,y_acc_offset,z_acc_offset,x_gyro_offset,y_gyro_offset,z_gyro_offset;
float xgyro=0.0;
float ygyro=0.0;
float zgyro=0.0; 
    
Serial pc(USBTX, USBRX); // tx, rx
nRF24L01P my_nrf24l01p(PTD6, PTD7, PTD5, PTD4, PTB20);    // mosi, miso, sck, csn, ce, irq

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);

char txData[TRANSFER_SIZE], rxData[TRANSFER_SIZE];
int txDataCnt = 0;
int rxDataCnt = 0;



FXOS8700Q_acc combo_acc(D14, D15, FXOS8700CQ_SLAVE_ADDR0);
FXOS8700Q_mag combo_mag(D14, D15, FXOS8700CQ_SLAVE_ADDR0);
FXAS21000 gyro(D14, D15);

DigitalOut red(LED_RED);
DigitalOut green(LED_GREEN);


void serial_callback(void);  // serial input from key board
void Motor_start(void);      // Motor start command   
void Motor_stop(void);       // Motor Stop command 
float Motor_control(float current, float pid, float rate);  //Motor Control function
void calibration ();
long distance;

HCSR04 sensor(D0, D1); 

int rf_link() {
    
        //printf("Function Call");
    
        // If we've received anything over the host serial link...
        if ( pc.readable() ) {
        
            // ...add it to the transmit buffer
            txData[txDataCnt++] = pc.getc();
            

            // If the transmit buffer is full
            if ( txDataCnt >= sizeof( txData ) ) {
                
                // Send the transmitbuffer via the nRF24L01+
                my_nrf24l01p.write( NRF24L01P_PIPE_P0, txData, txDataCnt );
                txDataCnt = 0;
            }

            // Toggle LED1 (to help debug Host -> nRF24L01+ communication)
            myled1 = !myled1;
        }
        // If we've received anything in the nRF24L01+...
        if ( my_nrf24l01p.readable() ) {

            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( NRF24L01P_PIPE_P0, rxData, sizeof( rxData ) );
               
            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {
               
                pc.putc( rxData[i] );
            }

            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled2 = !myled2;
        }
    }
    
   
int main()
{
    pc.baud(115200);

    my_nrf24l01p.powerUp();

    // Display the (default) setup of the nRF24L01+ chip
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n",  my_nrf24l01p.getRfFrequency() );
    pc.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress() );

    pc.printf( "Type keys to test transfers:\r\n  (transfers are grouped into %d characters)\r\n", TRANSFER_SIZE );

    my_nrf24l01p.setTransferSize( TRANSFER_SIZE );

    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();
    
    arm_pid_instance_f32 RPID;
    arm_pid_instance_f32 PPID;
    
    //Pitch
    PPID.Kp = PID_PITCH_KP/1000.0f;        /* Proporcional */
    PPID.Ki = PID_PITCH_KI/1000.0f;        /* Integral */
    PPID.Kd = PID_PITCH_KD/1000.0f;        /* Derivative */
    
    //Roll
    RPID.Kp = PID_ROLL_KP/1000.0f;        /* Proporcional */
    RPID.Ki = PID_ROLL_KI/1000.0f;        /* Integral */
    RPID.Kd = PID_ROLL_KD/1000.0f;        /* Derivative */
    
    arm_pid_init_f32(&RPID, 1);
    arm_pid_init_f32(&PPID, 1);
    GlobalTime.start();
    
    M1.period(0.02f);               //Comparten el mismo timer
    M1.pulsewidth(ESC1);
    M2.pulsewidth(ESC2);
    M3.pulsewidth(ESC3);
    M4.pulsewidth(ESC4);
    
    MotionSensorDataUnits adata;
    MotionSensorDataUnits mdata;
    //int16_t acc_raw[3];

    printf("\r\nStarting\r\n\r\n");
    
    combo_acc.enable();
    combo_mag.enable();
    printf("FXOS8700 Combo mag = %X\r\n", combo_mag.whoAmI());
    printf("FXOS8700 Combo acc = %X\r\n", combo_acc.whoAmI());

    printf("FXAS21000 Gyro = %X\r\n", gyro.getWhoAmI());
    
    wait(0.1);
    
    calibration();//calibration function call
    
    ProgramTimer.start();
    loopStartTime = ProgramTimer.read_us();
    timer = loopStartTime;
    
    rf_link();
    char start=rxData[0];
         
            //if (start=='s')
             
             
            // {
                printf(" Starting Motors...");
                wait(1);
                Motor_start();
                Flag=0;
                distance = sensor.distance();
                printf("distance  %d  \n",distance);
                if (distance<100)
             //{ 
              
              while(distance<100)
            {
               ESC1=Motor_control(ESC1,-pitch,0.000000f);
               ESC2=Motor_control(ESC2,-roll,0.000000f);
               ESC3=Motor_control(ESC3,roll,0.000000f);
               ESC4=Motor_control(ESC4,pitch,0.000000f);
               
               printf(" Stabalizing Quad  ");
               
               M1.pulsewidth(ESC1);
               M2.pulsewidth(ESC2);
               M3.pulsewidth(ESC3);
               M4.pulsewidth(ESC4);
               
               ESC1+=0.000001;
               ESC2+=0.000001;
               ESC3+=0.000001;
               ESC4+=0.000001;
               distance = sensor.distance();
               printf("distance  %d  \n",distance);
               printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1,ESC2,ESC3,ESC4);
               
               wait(1);
               
            } 
            
            if (rxData[0]=='x')
            {
                printf(" Motor stopped");
                Motor_stop();
                Flag=1;
            }
            printf("flag %d",Flag);
       
    
    while(1) 
    {   
        red = 0;
        combo_acc.getAxis(adata);
        adata.x=adata.x+x_acc_offset;   // Please test the sign
        adata.y=adata.y+y_acc_offset;
        adata.z=adata.z+z_acc_offset;
       // printf("FXOS8700 Acc:   X:%6.3f Y:%6.3f Z:%6.3f\r\n", adata.x, adata.y, adata.z);
        combo_mag.getAxis(mdata);
        //printf("FXOS8700 Mag:   X:%6.2f Y:%6.2f Z:%6.2f\r\n", mdata.x, mdata.y, mdata.z);

        gyro.ReadXYZ(gyro_data);
        gyro_data[0]=gyro_data[0]+x_gyro_offset;
        gyro_data[1]=gyro_data[1]+y_gyro_offset;
        gyro_data[2]=gyro_data[2]+z_gyro_offset;
       // printf("FXAS21000 Gyro: X:%6.2f Y:%6.2f Z:%6.2f\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
        
        R=sqrt(adata.x*adata.x + adata.y*adata.y + adata.z*adata.z);
        
        
// Sensor calibraiton Code here---- wont be required if kalman filters are used--------------------------------------------------------------
       
        
//  pitch angle calc------------------------------------------------------------------------
        acc_pitch_angle=180*atan(adata.x/(sqrt(adata.y*adata.y+adata.z*adata.z)))/3.14159;

        acc_roll_angle= 180*atan(adata.y/(sqrt(adata.x*adata.x+adata.z*adata.z)))/3.14159;

        acc_yaw_angle=180*atan(adata.z/(sqrt(adata.x*adata.x+adata.z*adata.z)))/3.14159;

        kalman_init(&filter_pitch, R_matrix, Q_Gyro_matrix, Q_Accel_matrix); 
        kalman_init(&filter_roll, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);

        kalman_predict(&filter_pitch, gyro_data[0],  (ProgramTimer.read_us() - timer)); 
        kalman_update(&filter_pitch, acos(adata.x/R));
        
        kalman_predict(&filter_roll, gyro_data[1],  (ProgramTimer.read_us() - timer)); 
        kalman_update(&filter_roll, acos(adata.y/R));

        angle[0] = kalman_get_angle(&filter_pitch);
        angle[1] = kalman_get_angle(&filter_roll);

        if (angle[0]>PI) angle[0] = PI;
        else if (angle[0]<0) angle[0] = 0.0f;
        else angle[0] += 0.0f;
        
        if (angle[1]>PI) angle[1] = PI;
        else if (angle[1]<0) angle[1] = 0.0f;
        else angle[1] += 0.0f;
        
        error_PITCH = angle[0] - (PITCH_ref+0.02);
        error_ROLL = angle[1] - (ROLL_ref+0.034); /// angle adjustment to remove offset 
        pitch = arm_pid_f32(&PPID, error_PITCH);
        roll = arm_pid_f32(&RPID, error_ROLL);
        
        if (Flag==0)
        {  
            printf("flag %d",Flag);
            rf_link();
         
            if (rxData[3]=='F')// throttle command
            {
                ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                ESC3 = Motor_control(ESC3,roll,0.0000000f);
                ESC4 = Motor_control(ESC4,pitch,0.0000000f);
                
                printf("Motor Forward");
                M1.pulsewidth(ESC1*0.8);
                M2.pulsewidth(ESC2*1.1); 
                M3.pulsewidth(ESC3*1.1); 
                M4.pulsewidth(ESC4*1.3);
                wait(0.1);
                printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*0.8,ESC2*1.1,ESC3*1.1,ESC4*1.3);
            }
            else if (rxData[3]=='B')
            {
                ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                ESC3 = Motor_control(ESC3,roll,0.0000000f);
                ESC4 = Motor_control(ESC4,pitch,0.0000000f);

                printf("Motor Reverse");
                M1.pulsewidth(ESC1*1.3);
                M2.pulsewidth(ESC2*1.1); 
                M3.pulsewidth(ESC3*1.1); 
                M4.pulsewidth(ESC4*0.8);
                wait(0.1);
                printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*1.3,ESC2*1.1,ESC3*1.1,ESC4*0.8);
            }
            else if (rxData[3]=='L')
            {
                ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                ESC3 = Motor_control(ESC3,roll,0.0000000f);
                ESC4 = Motor_control(ESC4,pitch,0.0000000f);

                printf("Motor Left");
                M1.pulsewidth(ESC1*1.1);
                M2.pulsewidth(ESC2*0.8);
                M3.pulsewidth(ESC3*1.3);
                M4.pulsewidth(ESC4*1.1);
                wait(0.1);
                printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*1.1,ESC2*0.8,ESC3*1.3,ESC4*1.1);
             }

            else if (rxData[3]=='R')
             {
                ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                ESC3 = Motor_control(ESC3,roll,0.0000000f);
                ESC4 = Motor_control(ESC4,pitch,0.0000000f);

                printf("Motor Right");
                M1.pulsewidth(ESC1*1.1);
                M2.pulsewidth(ESC2*1.3);
                M3.pulsewidth(ESC3*0.8);
                M4.pulsewidth(ESC4*1.1);
                wait(0.1);
                printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*1.1,ESC2*1.3,ESC3*0.8,ESC4*1.1);
              }

            else if (rxData[3]=='E')
              {
                ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                ESC3 = Motor_control(ESC3,roll,0.0000000f);
                ESC4 = Motor_control(ESC4,pitch,0.0000000f);

                printf("Motor Forward Right");                                   // 
                M1.pulsewidth(ESC1*0.8);
                M2.pulsewidth(ESC2*1.1);
                M3.pulsewidth(ESC3*0.8);
                M4.pulsewidth(ESC4*1.1);
                wait(0.1);
                printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*0.8,ESC2*1.1,ESC3*0.8,ESC4*1.1);
              }
            else if (rxData[3]=='Q')
              {
                ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                ESC3 = Motor_control(ESC3,roll,0.0000000f);
                ESC4 = Motor_control(ESC4,pitch,0.0000000f);

                 printf("Motor Forward Left");
                 M1.pulsewidth(ESC1*0.8);
                 M2.pulsewidth(ESC2*0.8);
                 M3.pulsewidth(ESC3*1.1);
                 M4.pulsewidth(ESC4*1.1);
                 wait(0.1);
                 printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*0.8,ESC2*0.8,ESC3*1.1,ESC4*1.1);
               }
            else if (rxData[3]=='C')
               {
                 ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                 ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                 ESC3 = Motor_control(ESC3,roll,0.0000000f);
                 ESC4 = Motor_control(ESC4,pitch,0.0000000f);

                 printf("Motor Back Right");
                 M1.pulsewidth(ESC1*1.1);
                 M2.pulsewidth(ESC2*1.1);
                 M3.pulsewidth(ESC3*0.8);
                 M4.pulsewidth(ESC4*0.8);
                 wait(0.1);
                 printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*1.1,ESC2*1.1,ESC3*0.8,ESC4*0.8);
                }
            else if (rxData[3]=='Z')
                {
                 ESC1 = Motor_control(ESC1,-pitch,0.0000000f);
                 ESC2 = Motor_control(ESC2,-roll,0.0000000f);
                 ESC3 = Motor_control(ESC3,roll,0.0000000f);
                 ESC4 = Motor_control(ESC4,pitch,0.0000000f);

                 printf("Motor Back Left");
                 M1.pulsewidth(ESC1*1.1);
                 M2.pulsewidth(ESC2*0.8);
                 M3.pulsewidth(ESC3*1.1);
                 M4.pulsewidth(ESC4*0.8);
                 wait(0.1);
                 printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1*1.1,ESC2*0.8,ESC3*1.1,ESC4*0.8);
                 }
                 
             else if (rxData[3]=='<')
             
             {  printf(" Rudder Left \n\t ");
                M1.pulsewidth(ESC1*0.8);
                M2.pulsewidth(ESC2*1.1);
                M3.pulsewidth(ESC3*1.1);
                M4.pulsewidth(ESC4*0.8);
                wait(0.1);
                
                ESC1 = Motor_control(ESC1,-pitch,0.000000f);
                ESC2 = Motor_control(ESC2,-roll,0.000000f);
                ESC3 = Motor_control(ESC3,roll,0.000000f);
                ESC4 = Motor_control(ESC4,pitch,0.000000f);
                //printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1,ESC2,ESC3,ESC4);
             }
             
             else if (rxData[3]=='>')
             
             {  printf(" Rudder Right\n\t ");
                M1.pulsewidth(ESC1*0.8);
                M2.pulsewidth(ESC2*1.1);
                M3.pulsewidth(ESC3*1.1);
                M4.pulsewidth(ESC4*0.8);
                wait(0.1);
                
                ESC1 = Motor_control(ESC1,-pitch,0.000000f);
                ESC2 = Motor_control(ESC2,-roll,0.000000f);
                ESC3 = Motor_control(ESC3,roll,0.000000f);
                ESC4 = Motor_control(ESC4,pitch,0.000000f);
               // printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1,ESC2,ESC3,ESC4);
             }
                     
            else 
               {
               
                printf(" Hover\n\t ");
                M1.pulsewidth(ESC1);
                M2.pulsewidth(ESC2);
                M3.pulsewidth(ESC3);
                M4.pulsewidth(ESC4);
                wait(0.1);
                
                ESC1 = Motor_control(ESC1,-pitch,0.000000f);
                ESC2 = Motor_control(ESC2,-roll,0.000000f);
                ESC3 = Motor_control(ESC3,roll,0.000000f);
                ESC4 = Motor_control(ESC4,pitch,0.000000f);
                printf("PWM1 =%f \t PWM2=%f \t PWM3=%f \t PWM4=%f ",ESC1,ESC2,ESC3,ESC4);
               }

                
        }
        timer = ProgramTimer.read_us();

        //printf("FXOS8700 Acc:   X:%6.3f Y:%6.3f Z:%6.3f\r\n", acc_pitch_angle, acc_roll_angle, acc_yaw_angle);
        printf(" Filtered angles : Pitch = %6.3f\t\t , Roll = %6.3f\t\t ",(angle[0]-PI/2)*Rad2Deg,(angle[1]-PI/2)*Rad2Deg); 
       
        //printf("\r\n");
        green = 1;
        wait(0.1);
    
}}
void Motor_start(void)
{
    Flag=0;
    printf(" Motor started\n\r ");
    ESC1 = 0.0006f;   //pitch up
    ESC2 = 0.0006f;   //roll  up
    ESC3 = 0.0006f;   //roll  down
    ESC4 = 0.0006f;   //pitch down
    for(int i=0;i<5;i++)
    {
        ESC1+=0.0001f;
        ESC2+=0.0001f;
        ESC3+=0.0001f;
        ESC4+=0.0001f;
        
        M1.pulsewidth(ESC1);  //modify fo flight
        M2.pulsewidth(ESC2);
        M3.pulsewidth(ESC3);
        M4.pulsewidth(ESC4);
        wait_ms(100);
    }
}
void Motor_stop(void){
    red = 0; green= 1; Flag = 1;
    printf(" Motor turning OFF \n\r");
    while ((ESC1 > 0.0006f)||(ESC2 > 0.0006f)||(ESC3 > 0.0006f)||(ESC4 > 0.0006f))
    {
        ESC1 -= 0.0001f;
        ESC2 -= 0.0001f;
        ESC3 -= 0.0001f;
        ESC4 -= 0.0001f;
        M1.pulsewidth(ESC1);
        M2.pulsewidth(ESC2);
        M3.pulsewidth(ESC3);
        M4.pulsewidth(ESC4);
        wait_ms(250);   
    }
    if (ESC1 < 0.0006f) ESC1 = 0.0006f;
    if (ESC2 < 0.0006f) ESC2 = 0.0006f;
    if (ESC3 < 0.0006f) ESC3 = 0.0006f;
    if (ESC4 < 0.0006f) ESC4 = 0.0006f;
}
float Motor_control(float current, float pid, float rate)
{
        if ((current + pid + rate)>0.0020f) return 0.0020f;
        else if ((current + pid + rate)<0.001f) return 0.001f; 
        else return current + pid + rate;
}

void calibration()
{       MotionSensorDataUnits adata;
        MotionSensorDataUnits mdata;
        combo_acc.enable();
        combo_mag.enable();
        printf("Starting Offset calculation.....\n");
        wait(0.10);
        combo_acc.getAxis(adata);
        combo_mag.getAxis(mdata);
        gyro.ReadXYZ(gyro_data);
        xaccel=adata.x;
        yaccel=adata.y;
        zaccel=adata.z;
        for (int j=0;j<2000;j++)
       {
        xaccel+=adata.x;
        yaccel+=adata.y;
        zaccel+=adata.z;
        xgyro+=gyro_data[0];
        ygyro+=gyro_data[1];
        zgyro+=gyro_data[2];
        
        }
        
        
        xmean=xaccel/2000;
        ymean=yaccel/2000;
        zmean=zaccel/2000;
        xGmean=xgyro/2000;
        yGmean=ygyro/2000;
        zGmean=zgyro/2000;
        
        x_acc_offset=-xmean/8;
        y_acc_offset=-ymean/8;
        z_acc_offset=(1-zmean)/8;
        x_gyro_offset=-xGmean/4;
        y_gyro_offset=-yGmean/4;
        z_gyro_offset=-zGmean/4;
        printf("Offset calculation completed ....\n");
        wait(0.10);
        }



    
    

