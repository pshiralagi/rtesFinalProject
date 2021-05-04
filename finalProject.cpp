/* ========================================================================== */
/*                
// Antara Prakash and Pavan Shiralagi
// RTES final project                                                            */
//  References -
// Sam Siewert, December 2017
//	http://mercury.pr.erau.edu/~siewerts/cec450/code/RT-Clock/posix_clock.c
//  http://www.piprojects.xyz/ultrasonic-distance-sensor/
//
//  This code contains a COVID room entry safety detector based on the following services
//
//  Sequencer runs ultrasonic function, contains inter-process communication and schedules the two services
//              Service_1 for ultrasonic sensor reading
//              Service_2 camera frame aquisition, circle detection
//
//
//
// Sequencer - 30 Hz 
//                   [gives semaphores to all other services]
// Service_1 - 3 Hz  , every 10th Sequencer loop
//                   [camera frame aquisition]
// Service_2 - 1 Hz  , every 30th Sequencer loop 
//                   [time-stamp middle sample image with cvPutText or header]
//
// With the above, priorities by RM policy would be:
//
// Sequencer = RT_MAX	@ 50 Hz
// Servcie_1 = RT_MAX-1	@ 3 Hz
// Service_2 = RT_MAX-2	@ 2 Hz
// Speaker runs as a best effort service
//
// Here are a few hardware/platform configuration settings on your Jetson
// that you should also check before running this code:
//
// 1) Check to ensure all your CPU cores on in an online state.
//
// 2) Check /sys/devices/system/cpu or do lscpu.
//
//    Tegra is normally configured to hot-plug CPU cores, so to make all
//    available, as root do:
//
//    echo 0 > /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable
//    echo 1 > /sys/devices/system/cpu/cpu1/online
//    echo 1 > /sys/devices/system/cpu/cpu2/online
//    echo 1 > /sys/devices/system/cpu/cpu3/online
//
// 3) Check for precision time resolution and support with cat /proc/timer_list
//
// 4) Ideally all printf calls should be eliminated as they can interfere with
//    timing.  They should be replaced with an in-memory event logger or at
//    least calls to syslog.
//
// 5) For simplicity, you can just allow Linux to dynamically load balance
//    threads to CPU cores (not set affinity) and as long as you have more
//    threads than you have cores, this is still an over-subscribed system
//    where RM policy is required over the set of cores.

// This is necessary for CPU affinity macros in Linux
#define _GNU_SOURCE

// Setup for seqgen, seqgen2x and other 100x
#define seqgen
//#define seqgen2
//#define seqgen100


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>
#include <pigpio.h>
#include <fcntl.h>

#include <syslog.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <errno.h>

using namespace cv;
using namespace std;

#define HRES 640
#define VRES 480

#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (1)
#define TRUE (1)
#define FALSE (0)
#define TRIG 18
#define ECHO 22

#define LOW 500000
#define MEDIUM 250000
#define HIGH 50000
#define OFF 0
#define PWM_pin 17


#define NUM_THREADS (3+1)

#ifdef seqgen
int freq = 40000000;
#endif

#ifdef seqgen2
int freq = 16666666;
#endif

#ifdef seqgen100
int freq = 33333333.33;
#endif

int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE;
sem_t semS1, semS2, semS3, semRT, semSPKR;
struct timeval start_time_val;
uint8_t RT_ON, SPKR_CODE;
uint32_t frequency;


struct timespec start_time = {0, 0};
struct timespec end_time = {0, 0};
struct timespec time_diff = {0, 0};

struct timespec temp = {0, 0};

struct timespec wcet1 = {0, 0};
struct timespec wcet2 = {0, 0};
struct timespec wcet3 = {0, 0};


typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

typedef struct {
	int unique_id;
	int tmp;
} data;


void *Sequencer(void *threadp);
uint8_t ultrasoinc_init(void);
void *Service_1(void *threadp);
void *Service_2(void *threadp);
void *Service_3(void *threadp);
double getTimeMsec(void);
void print_scheduler(void);

void ipc_init(void);
void ipc_alarm(int tmp);

void *Service_3(void *threadp){
    struct timeval current_time_val;
    double current_time;
    int dist;
    int i=0;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Frame Sampler thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Frame Sampler thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abortS1)
    {
        sem_wait(&semS1);
        clock_gettime(CLOCK_REALTIME, &start_time);
        S3Cnt++;

        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "S3 Frame Sampler release %llu @ sec=%d, msec=%d\n", S3Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

        if(frequency == OFF){
        gpioWrite(PWM_pin, PI_OFF);
        }else if(frequency == HIGH){
        while(i < 100){
        gpioWrite(PWM_pin, PI_ON);
        gpioDelay(HIGH);
        gpioWrite(PWM_pin, PI_OFF);
        gpioDelay(HIGH);
        i++;
        }
        frequency = OFF;

        }else{
        gpioWrite(PWM_pin, PI_ON);
        gpioDelay(frequency);
        gpioWrite(PWM_pin, PI_OFF);
        gpioDelay(frequency);
        }
        i = 0;
        
        clock_gettime(CLOCK_REALTIME, &end_time);
        delta_t(&end_time, &start_time, &time_diff);
    
        temp = time_diff;
        delta_t(&temp, &wcet3, &time_diff);
        
        if((time_diff.tv_sec) >= 0)
        {
            wcet3 = temp;
        }
    }
    pthread_exit((void *)0);
}

void setup() {
        gpioCfgSetInternals(1<<10);
        gpioInitialise();
        gpioSetMode(TRIG, PI_OUTPUT);
        gpioSetMode(ECHO, PI_INPUT);
        gpioSetMode (PWM_pin, PI_OUTPUT);
        //TRIG pin must start LOW
        gpioWrite(TRIG, PI_OFF);
        gpioDelay(30); // it is 30 microsecond should be 30 millisecond
}

int getCM() {
        //Send trig pulse
        gpioWrite(TRIG, PI_ON);
        gpioDelay(20);
        gpioWrite(TRIG, PI_OFF);

        //Wait for echo start
        while(gpioRead(ECHO) == PI_OFF);

        //Wait for echo end
        long startTime = gpioTick();
        while(gpioRead(ECHO) == PI_ON);
        long travelTime = gpioTick() - startTime;

        //Get distance in cm
        int distance = travelTime / 58;

        return distance;
}

int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  if(dt_sec >= 0)
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
  else
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }

  return(1);
}

int main(void)
{
    struct timeval current_time_val;
    int i, rc, scope;
    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;
    RT_ON = 0;
    frequency = OFF;
    
    printf("Starting Sequencer Demo\n");
    gettimeofday(&start_time_val, (struct timezone *)0);
    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

   printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));

   setup();
   ipc_init();
   


    // initialize the sequencer semaphores
    //
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semRT, 0, 1)) { printf ("Failed to initialize SRT semaphore\n"); exit (-1); }
    if (sem_init (&semSPKR, 0, 1)) { printf ("Failed to initialize SPKR semaphore\n"); exit (-1); }

    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();
    ipc_alarm(OFF);

    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    for(i=0; i < NUM_THREADS; i++)
    {

      CPU_ZERO(&threadcpu);
      CPU_SET(3, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1 = RT_MAX-1	@ 2.5 Hz
    //
    rt_param[1].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1,                 // thread function entry point
                      (void *)&(threadParams[1]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");


    // Service_2 = RT_MAX-2	@ 1 Hz
    //
    rt_param[2].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_2, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");

    rt_param[3].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], Service_3, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        printf("pthread_create successful for service 3\n");

    // Wait for service threads to initialize and await release by sequencer.
    //
    // Note that the sleep is not necessary of RT service threads are created wtih 
    // correct POSIX SCHED_FIFO priorities compared to non-RT priority of this main
    // program.
    //
    // usleep(1000000);
 
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");
    threadParams[0].sequencePeriods=900;

    // Sequencer = RT_MAX	@ 50 Hz
    //
    rt_param[0].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequeencer service 0\n");


   for(i=0;i<NUM_THREADS;i++)
       pthread_join(threads[i], NULL);

   printf("\nTEST COMPLETE\n");
   return 0;

}


void *Sequencer(void *threadp)
{
    struct timeval current_time_val;
    struct timespec delay_time = {0,freq}; // delay based on define
    struct timespec remaining_time;
    double current_time;
    double residual;
    uint8_t ultra_return;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    do
    {
        delay_cnt=0; residual=0.0;

        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

        seqCnt++;
        gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);


        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);

        if (RT_ON == 0)
        {
            ultra_return = ultrasoinc_init();
            sem_post(&semS3);
            sem_wait(&semSPKR);
            SPKR_CODE = 0;
            ipc_alarm(SPKR_CODE);
            frequency = OFF;
            sem_post(&semSPKR);  
            if (ultra_return == 1)
            {
                sem_wait(&semRT);
                RT_ON = 1;
                sem_post(&semRT);
                sem_wait(&semSPKR);
                SPKR_CODE = 1;
                ipc_alarm(SPKR_CODE);
                frequency = LOW;
                sem_post(&semSPKR);

            }
        }

        if (RT_ON == 1)
        {

        
            // Release each service at a sub-rate of the generic sequencer rate
    #ifdef seqgen
            // Servcie_1 = RT_MAX-1	@ 2 Hz
            if((seqCnt % 4) == 0) sem_post(&semS1);

            // Service_2 = RT_MAX-2	@ 1 Hz
            if((seqCnt % 5) == 0) sem_post(&semS2);
            else
            {
                sem_post(&semS3);
            }
    #endif

    #ifdef seqgen2
            // Servcie_1 = RT_MAX-1	@ 30 Hz
            if ((seqCnt % 2) == 0) sem_post(&semS1);

            // Service_2 = RT_MAX-2	@ 10 Hz
            if ((seqCnt % 6) == 0) sem_post(&semS2);
            else
            {
                sem_post(&semS3);
            }
    #endif

    #ifdef seqgen100
            // Servcie_1 = RT_MAX-1	@ 300 Hz
            if ((seqCnt % 10) == 0) sem_post(&semS1);

            // Service_2 = RT_MAX-2	@ 100 Hz
            if ((seqCnt % 30) == 0) sem_post(&semS2);
            else
            {
                sem_post(&semS3);
            }

    #endif

            //gettimeofday(&current_time_val, (struct timezone *)0);
            //syslog(LOG_CRIT, "Sequencer release all sub-services @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

        }
    } while(!abortTest);

    sem_post(&semS1); sem_post(&semS2);sem_post(&semS2)
    abortS1=TRUE; abortS2=TRUE, abortS3=TRUE;

    pthread_exit((void *)0);
}


uint8_t ultrasoinc_init(void)
{
    static int dist;
    dist = getCM();
    if (dist < 150 && dist > 50)
    {
        return 1;
    }
    return 0;
}
void *Service_1(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    int dist;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Frame Sampler thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Frame Sampler thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abortS1)
    {
        sem_wait(&semS1);
        clock_gettime(CLOCK_REALTIME, &start_time);
        S1Cnt++;

        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "S1 Frame Sampler release %llu @ sec=%d, msec=%d\n", S1Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

        //get distance, if distance is less than some value, sound alarm and switch off RT for camera and ultra
        dist = getCM();
        syslog(LOG_CRIT, "Distance = %d", dist);
        if ((dist < 120) && (dist > 70))
        {
            sem_wait(&semSPKR);
            SPKR_CODE = 2;
            ipc_alarm(SPKR_CODE);
            frequency = MEDIUM;
            sem_post(&semSPKR);
        }
        if (dist <= 70)
        { 
            sem_wait(&semSPKR);
            SPKR_CODE = 3;
            ipc_alarm(SPKR_CODE);
            frequency = HIGH;
            sem_post(&semSPKR);  
            sem_wait(&semRT);
            RT_ON = 0;
            sem_post(&semRT);
        }
        
        clock_gettime(CLOCK_REALTIME, &end_time);
        delta_t(&end_time, &start_time, &time_diff);
    
        temp = time_diff;
        delta_t(&temp, &wcet1, &time_diff);
        
        if((time_diff.tv_sec) >= 0)
        {
            wcet1 = temp;
        }
    }
    pthread_exit((void *)0);
}


void *Service_2(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S2Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    int circ_detected = 0;
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(0);
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(argv[1]);
    CvCapture* capture;
    IplImage* frame;
    int dev=0;
    Mat gray;
    vector<Vec3f> circles;

    capture = (CvCapture *)cvCreateCameraCapture(dev);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Time-stamp with Image Analysis thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Time-stamp with Image Analysis thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abortS2)
    {
        sem_wait(&semS2);
        clock_gettime(CLOCK_REALTIME, &start_time);
        S2Cnt++;

        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "S2 Time-stamp with Image Analysis release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        
        //read from camera, compare circle diameter, if match, switch off RT and send safe to speaker. Else, repeat
        frame=cvQueryFrame(capture);
        if(!frame) break;

        Mat mat_frame(cvarrToMat(frame));
        
        // Does not work in OpenCV 3.1
        //cvtColor(mat_frame, gray, CV_BGR2GRAY);
        cvtColor(mat_frame, gray, COLOR_BGR2GRAY);

        GaussianBlur(gray, gray, Size(9,9), 2, 2);

        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 100, 50, 0, 0);

        // printf("circles.size = %ld\n", circles.size());

        for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
        //   printf("Radius -- - -- %d\n\r", radius);
          // circle center
          circle( mat_frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          circle( mat_frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
          if ((radius < 130) && (radius > 60))
          {
              circ_detected = 1;
          }
          circles[i][0] = 0;
          circles[i][1] = 0;
          circles[i][2] = 0;
          
        }

        // Does not work in OpenCV 3.1
        //cvShowImage("Capture Example", frame);

        imshow("Capture Example", mat_frame);

        if (circ_detected == 1)
        {
            sem_wait(&semSPKR);
            SPKR_CODE = 0;
            frequency = OFF;
            ipc_alarm(SPKR_CODE);
            sem_post(&semSPKR);
            sem_wait(&semRT);
            RT_ON = 0;
            sem_post(&semRT);
        }
        clock_gettime(CLOCK_REALTIME, &end_time);
        delta_t(&end_time, &start_time, &time_diff);
    
        temp = time_diff;
        delta_t(&temp, &wcet2, &time_diff);
        
        if((time_diff.tv_sec) >= 0)
        {
            wcet2 = temp;
        }
        circ_detected = 0;
        circ_detected = 0;
        char c = cvWaitKey(10);
        if( c == 'q' ) break;
        // printf("Thread 1, WCET : %ld sec, %ld msec, %ld microsec, %ld nanosec\n\n\r", wcet1.tv_sec, (wcet1.tv_nsec / NSEC_PER_MSEC), (wcet1.tv_nsec / NSEC_PER_MICROSEC), wcet1.tv_nsec);
        // printf("Thread 2, WCET : %ld sec, %ld msec, %ld microsec, %ld nanosec\n\n\r", wcet2.tv_sec, (wcet2.tv_nsec / NSEC_PER_MSEC), (wcet2.tv_nsec / NSEC_PER_MICROSEC), wcet2.tv_nsec);
        // printf("Thread 3, WCET : %ld sec, %ld msec, %ld microsec, %ld nanosec\n\n\r", wcet3.tv_sec, (wcet3.tv_nsec / NSEC_PER_MSEC), (wcet3.tv_nsec / NSEC_PER_MICROSEC), wcet3.tv_nsec);
    }

    pthread_exit((void *)0);
}


double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}


void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

/*	Function to initialize shared memory	*/
void ipc_init(void)
{
	int fd_shared;
	fd_shared = shm_open("shareTmp",  O_CREAT | O_RDWR, 0666);
	if (fd_shared < 0)
	{
		perror("open");
	}
	ftruncate(fd_shared, 500);
	if (close(fd_shared) < 0)
	{
		perror("close");
	}
}

/*	Function to store temperature in shared memory	*/
void ipc_alarm(int tmp)
{
	data temperature = {1, tmp};
	data *temperature_ptr = &temperature;
	data *temp_ptr = NULL;
	int fd_shared;
	fd_shared = shm_open("shareTmp", O_RDWR, 0666);
	if (fd_shared < 0)
	{
		perror("open");
	}
	temp_ptr = (data *)mmap(NULL, sizeof(data), PROT_WRITE, MAP_SHARED, fd_shared, 0);
	if (close(fd_shared) < 0)
	{
		perror("close");
	}
	memcpy((void *)(&temp_ptr[0]),(void*)temperature_ptr,sizeof(data));
	munmap(temp_ptr,sizeof(data));
}