// Set (CMAKE_CXX_FLAGS "-lwiringPi")
#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <string> 
#include <stdlib.h>
#include <cstdlib>
#include <sstream>
#include <thread>
#include <mutex> 
#include <ctime>
#include <sys/time.h>
#include <unistd.h> 
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
 
using namespace cv; 
using namespace std;
std::mutex mtxcam;
std::mutex mtxposis;
std::mutex cal_pwm;
std::mutex aciona_servo;
std::mutex trava_emerg;

int fd;
int servoMinPos = 800;
int servoMaxPos = 2000;
int servoMinCh = 0;
int servoMaxCh = 31;
double deadline = 0;
int count_tic = 1;
int count_toc = 1;
int y = 0;


//temporizado = uma estrurura para contar tempo de execução de funções 
struct
{
  struct timeval time;
  struct timeval timereset;
} tictocctrl;

void tic(void)
{
  gettimeofday(&tictocctrl.timereset, NULL);
}

double toc(void)
{
  gettimeofday(&tictocctrl.time, NULL);
  return ((tictocctrl.time.tv_sec - tictocctrl.timereset.tv_sec) + (tictocctrl.time.tv_usec - tictocctrl.timereset.tv_usec) * 1e-6);
}
//

// Gabriel ===================================================>

//Convert integer to string
std::string int_to_string(int n){
  std::ostringstream str1;
  str1 << n;
  std::string str = str1.str();
  return str1.str();
}

int verifyBounds(int ch, int pos){
  if (ch > servoMaxCh or ch < servoMinCh)
  {
    std::cout << "Channel is not within its boundaries\n";
    return 0;
  }
  if (pos > servoMaxPos or pos < servoMinPos)
  {
    std::cout << "Position is not within its boundaries\n";
    return 0;
  }
  return 1;
}


int openPort(void){
  int fd; /* File descriptor for the port */
  struct termios options;
  int baud = B115200;

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0)
  {
    perror("openPort: Unable to open /dev/ttyUSB0\n"); //It coud not open port
  }
  else
    fcntl(fd, F_SETFL, 0);

  memset(&options, 0, sizeof(options));
  cfsetispeed( &options, baud );
  cfsetospeed( &options, baud );

  options.c_iflag = IGNBRK | IGNPAR;
  options.c_oflag = 0;
  options.c_cflag |= CREAD | CS8 | CLOCAL;
  options.c_lflag = 0;

  if( tcsetattr( fd, TCSANOW, &options ) < 0 )
  {
    printf( "ERROR: setting termios options\n" );
    return false;
  }

  // Make sure queues are empty
  tcflush( fd, TCIOFLUSH );

  printf( "Successfully opened port /dev/ttyUSB0\n");

  return (fd);
}

// Move the ch servo to postition pos
void moveServo(int ch, int pos){
  if(verifyBounds(ch, pos) == 1)
  {
    int n;
    std::string str = '#' + int_to_string(ch) + " P" + int_to_string(pos) + '\r';
    const char * msg = str.c_str();
    n = write(fd, msg, strlen(msg));
    if (n < 0)
      fputs("write() failed!\n", stderr);
  }
}

// Centralizes all servos
void setServos(int num) {
  int NUM_ACTIVE_SERVOS = num;

  for(int i = 1; i <= NUM_ACTIVE_SERVOS; i++) {
    moveServo(i-1, 1500);
    sleep(0.1); //give each servo initial wait time
  }
}

//=============================================================>

//função para ler posição do objeto //

int* ler_posicao_objeto(Mat src, int ant_x, int ant_y, int erro_x, int erro_y){
    int* v_o = new int[2];
    char str1[20] = "ALVO TRAVADO!";
    char str2[20] = "ALVO DESTRAVADO!";
    v_o[0] = ant_x;
    v_o[1] = ant_y;
    Mat dilation_dst;
    int dilation_elem = 15;
    int dilation_size = 15;
    Mat canny, hsv, black;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cvtColor(src, hsv, COLOR_BGR2HSV);
   inRange(hsv, Scalar(0,155, 100), Scalar(10,255 , 255), black);

    // ====================================================
    //dilatação

  Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate(black, dilation_dst, element );

  ///////////////////////////////////////////////////////

    Canny(dilation_dst, canny, 50, 150, 3);

    findContours(canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));


    // pega o momento
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
      mu[i] = moments(contours[i], false);
    }

    // pega o centro do objeto.
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
      mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    // desenha
  
    Mat drawing(canny.size(), CV_8UC3, Scalar(255, 255, 255));
      
    for (int i = 0; i < contours.size(); i++)
    {
     double a = contourArea(contours[i], false);
     //cout << a << endl;
     Scalar color1;
      if (a > 3000)
      {
       // cout << erro_x << "||" <<erro_y << endl;
        if ((erro_x < 20 & erro_x > -20)){
           if ((erro_y < 20 & erro_y > -20)){
             //fprintf(fr,"\n%f", 1.9);
             count_toc = 2;
             count_tic = 0;
             color1 = Scalar(0, 255, 0); // B G R values
             putText(src, str1 , cvPoint(5,470), 
             FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,255,0), 1, CV_AA);
           }else {color1 = Scalar(0, 0, 255);
                  //deadline = deadline + toc();
                  //fprintf(fr,"\n%f", deadline);
                  count_toc = 1;
                  count_tic = 2;
                  y = 0;
                  putText(src, str2 , cvPoint(5,470), 
                  FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,0,255), 1, CV_AA);
                  }
        }else {color1 = Scalar(0, 0, 255);
                  //deadline = deadline + toc();
                  //fprintf(fr,"\n%f", deadline);
                  count_toc = 1;
                  count_tic = 2;
                  y = 0;
                  putText(src, str2 , cvPoint(5,470), 
                  FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,0,255), 1, CV_AA);
                  } // B G R values
        Scalar color2 = Scalar(255, 0, 0); // B G R values
       // Scalar color1 = Scalar(0, 255, 0); // B G R values
        drawContours(src, contours, i, color1, 2, 8, hierarchy, 0, Point());
        circle(src, mc[i], 5, color2, -1, 8, 0);
        v_o[0] =(int)mc[i].x;
        v_o[1] =(int)mc[i].y; 

      }
    }
    
    //cout << "oX = " << v_o[0] << "oY = "<< v_o[1] << endl;
   imshow("DEASHBOARD", src);
   return v_o;
}

// função para ler posição do laser//
int* ler_posicao_laser(Mat src, int ant_lx, int ant_ly ){
    int* v_l = new int[2];
    v_l[0] = ant_lx;
    v_l[1] = ant_ly;
    Mat dilation_dst;
    int dilation_elem = 5;
    int dilation_size = 5;
    Mat canny, hsv, black;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0,0, 207), Scalar(14, 36, 255), black);

        // ====================================================
    //dilatação

  Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate(black, dilation_dst, element );

  ///////////////////////////////////////////////////////

    Canny(dilation_dst, canny, 50, 150, 3);

    findContours(canny, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));


    // pega o momento
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)  
    {
      mu[i] = moments(contours[i], false);
    }

    // pega o centro do objeto.
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
      mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    // desenha

   // Mat drawing(canny.size(), CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < contours.size(); i++){
        Scalar color1 = Scalar(0, 255, 0); // B G R values
        circle(src, mc[i], 2, color1, -1, 8, 0);
        v_l[0] = (int)mc[i].x;
        v_l[1] = (int)mc[i].y;
    }
    //cout << " X = " << v_l[0] << " Y = "<< v_l[1] << endl;
    //imshow("laser", src);
     return v_l;
}

//função que calcula erro e retorna largura de pulso
int* calcula_pulso_pwm(int e_x, int e_y, int pwm_x, int pwm_y){
      
      float kp = 0.1;
      int* pwm_atual = new int[2];
      pwm_atual[0] = pwm_x;
      pwm_atual[1] = pwm_y;

         pwm_atual[0] = pwm_x -(int)(kp*e_x);
         if(pwm_atual[0] > 2000){
           pwm_atual[0] = 2000;
         }
        if(pwm_atual[0] < 1000){
           pwm_atual[0] = 1000;
         }

      //PWM para y
         pwm_atual[1] = pwm_y -(int)(kp*e_y);
         if(pwm_atual[1] > 2000){
           pwm_atual[1] = 2000;
         }
        if(pwm_atual[1] < 1000){
           pwm_atual[1] = 1000;
         }
         

      return pwm_atual;

}
// Função que aciona os servos motores
void acionar_servo(int pwm_x, int pwm_y){
  //cout << pwm_x << "*" << pwm_y << endl;
  moveServo(1,pwm_x);
  moveServo(2,pwm_y);
  
}
// função principal //
int main(){
  int fd1 = openPort();
  fd = fd1;
     setServos(2);
    // setup da webcam.
    VideoCapture cap(1);
    //Verificação se a camera foi aberta. 
    if(!cap.isOpened()){
         cout << "Erro ao abrir a camera" << endl;
         return -1;
      }
  // Definição das dimenssões do frame capturado.
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  // Declaração das variáveis para receber a posição do laser e do objeto e o offset dos servo motores.    
  int laser_x, laser_y;
  int objeto_x = 200, objeto_y = 200;
  int pwm_x = 1600;
  int pwm_y = 1600;
  int erro_x = 30;
  int erro_y = 30;
  
  float t1 = 0, t2 = 0,t3 = 0, t4 = 0;   
  //abrir um aquivo para pegar os tempos 
  	FILE *fp;
    //FILE *fr;
    char filename[20]= "tracker.csv";
    //char filename1[20]= "deadline.csv";
	  fp=fopen(filename,"w+");
    //fr=fopen(filename1,"w+");
	  //fprintf(fp,"tempo\n");
    //fprintf(fr,"tempo\n");
  // Declaração dos ponteiros para as função que captura as posições do objeto, do laser, e largura do pwm. 
  int* v_laser;
  int* v_objeto;
  int* pwm;
  float x = 1.223;
  // loop para captura dos frames.
  while (1){
    // declaração da imagem de entrada src.
    //tic();
    Mat src;
    // vou colocar um mutex aqui
    mtxcam.lock();
    cap >> src;
   // verificação se a imagem tem conteudo.
   if (src.empty()){
    break;
   }
   // fim do mutex 
   mtxcam.unlock();
   //t1 = toc();

   ////////////////////////////////////////////////////////////////////////
   //Funções para captura das posições, tanto do laser quanto do objeto.
   // as duas

   /*  
   if(count_tic == 1){
      tic(); // tic para contar o tempo até o deadline de encontrar o objeto.
   }
   
   if(count_toc == 2){
     //cout << count_toc << endl;
     if(y == 0){
     x = toc();
     fprintf(fp,"\n%f",x);
     y = 1;
     tic();
     }
     if(x > 5){
       printf("\nVOCÊ MORREU\n");
       close(fd);
       fclose(fp);
       exit(0);
     }

   }
*/
   //tic();
   mtxposis.lock();
   v_objeto = ler_posicao_objeto(src, objeto_x, objeto_y, erro_x, erro_y);
   v_laser = ler_posicao_laser(src, laser_x, laser_y);

   //Coordenada do objeto
   objeto_x = v_objeto[0];
   objeto_y = v_objeto[1];
   //Coordenada do 
   laser_x = v_laser[0];
   laser_y = v_laser[1];
   // Calculo erro
   erro_x = objeto_x - laser_x;  
   erro_y = objeto_y - laser_y; 
   //printf("\nex %i | ey %i", erro_x, erro_y);
   mtxposis.unlock();
   //t2 = toc();
   ///////////////////////////////////////////////////////////////
   //tic();
   cal_pwm.lock();
   // Função para calcular o erro e tranforma em largura de pulso
   pwm = calcula_pulso_pwm(erro_x, erro_y, pwm_x, pwm_y);
   // valores de de lagura de pulso para os servos motores 
   pwm_x = pwm[0];
   pwm_y = pwm[1];
   cal_pwm.unlock();
   //t3 = toc();
   ///////////////////////////////////////////////////////////////
   // Função para acionar os dois servo motores
   //tic();
   aciona_servo.lock();
   acionar_servo(pwm_x, pwm_y);
   aciona_servo.unlock();
   //t4 = toc();
    ////////////////////////////////////////////////////////////
    // Trava de seguraça mais butão de emergência
    trava_emerg.lock();
    char c = (char)waitKey(1);
    if (c == 32){
       fclose(fp);
      while (1){
       char c = (char)waitKey(1);
       if (c == 32){
          break;
       }
       if (c == 27){
           destroyAllWindows();
           close(fd);
           fclose(fp);
           //fclose(fr);
           exit(0);
       }
      }
    }
    trava_emerg.unlock();
    //fprintf(fp,"\n%f ", x);
   // fprintf(fp,"\n%f %f %f %f",t1, t2, t3, t4);
    //printf("\n%f | %f | %f | %f",t1, t2, t3, t4);
  }
  // fecha a porta serial
  close(fd);
  fclose(fp);
  //fclose(fr);
//Fecha a janela
  destroyAllWindows();
  return 0;
}