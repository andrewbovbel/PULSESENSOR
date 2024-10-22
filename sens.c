#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h> 



#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <netdb.h>



#include <math.h>


/*-------------------------------------- */
//Global Stuff 

int g_range; //0=2g 1=4g  2=8g 3=16g (I think these are the possible ones ...)

#define NUM_SENSOR 4

#define Q_SIZE  65536
#define BUF_LEN 4096

typedef struct sensor{

        int id;

        int fd; //socket file discriptor

        // DATA BUFFERS  forvalues
	float* x;
	float* y;
	float* z;
	int queue_write;
	int queue_count;
	//mutex and condition variable for  above buffer
	//(even we do never have the consumer write the variable here ..
	pthread_mutex_t w_lock; ///write pos lock

	pthread_mutex_t d_lock; ///write pos lock
	pthread_cond_t  dava_rdy; // Signaled if "sufficient" data avaiable to process 

	// Buffer for socket read
	int buf_pos;
	int buf_num;
	unsigned char buf[BUF_LEN];

	//Trigger positions for saving
	int trig_save_start_pos; // -1 if not set, 


	
}sensor;

/*-------------------------------------- */
sensor* make_sensor(int id)
{
	sensor* sen=(sensor*)malloc(sizeof(sensor));
	sen->id=id;
	sen->x=(float*)malloc(Q_SIZE*sizeof(float));
	sen->y=(float*)malloc(Q_SIZE*sizeof(float));
	sen->z=(float*)malloc(Q_SIZE*sizeof(float));
	//Space for data
	sen->queue_write=0; // wirte pos
	sen->queue_count=1; //read pos

	sen->buf_pos=0;
	sen->buf_num=0;
	// Socket read buffer is static

	//sen->d_lock=PTHREAD_MUTEX_INITIALIZER;
	//sen->w_lock=PTHREAD_MUTEX_INITIALIZER;
	//sen->dava_rdy=PTHREAD_COND_INITIALIZER; 


	return(sen);
};

sensor* sensors[NUM_SENSOR]; // We have only 4 sensors 

//--------------------------------------------------------------------------
// Check if we need to trigger !!
// we trigger on one sensor i on axis  1,2,3
#define NUM_TRI_WIN 32
int trig_sen_id=0;
int trig_sen_dir=0;


int trigger_armed;
int trigger_save_file;

int trigger_queue_pos;

float last_val[NUM_TRI_WIN];
int last_pos=0; 
float trig_val;
float trig_threshhold;



// WE HAVE TO INIT THIS !!!


void init_trigger(int id,float thre)
{

	trigger_armed=1;
	trigger_save_file=0;
	trig_sen_id=id;
	for(int i=0;i<NUM_TRI_WIN;i++)last_val[i]=0.;
	last_pos=0;
	trig_val=0.;
	// the - NUM_TRI_WIN removes DC !!
	trig_threshhold=thre/NUM_TRI_WIN+NUM_TRI_WIN;


	printf("Trigger Armed \n");
}


int check_trigger(float x,float y, float z)
{
	if(trigger_armed==0)return(0);
	//if(sen->id==trig_sen_id){
		last_pos=(last_pos+1)%NUM_TRI_WIN;
	 	trig_val=trig_val-last_val[last_pos]; // Remove OLD
		const float  add_val= x*x+y*y+z*z;
		trig_val+=add_val;// fabsf(x);// Depends on direction !!
		last_val[last_pos]=add_val;
	//}

	// Depemds on G range, 
	if(trig_val> trig_threshhold){
		printf("TRIGGER %f \a\n",trig_val);
		trig_val=0;
		trigger_armed=0;
		trigger_save_file=1;

		return(1); // e.g. NUM_TRI_WIN with average of .5
	}
	return(0);
}

//write the latest num values to the disk x y z
// up to Q_SIZE 
// file name constructed from sensor ID and the r_nr as S'ID'_'r_nr' 
// (the r_nr allows to d several recordings per sensor ...

// We will also DC filter it, so a two pass process
void write_disk(sensor* sen,int r_nr, int num)
{
	int i,idx;
	char buff[32];
	float mean_x=0;
	float mean_y=0;
	float mean_z=0;
	sprintf(buff,"S%d_%d",sen->id,r_nr);
	FILE* out_file=fopen(buff,"w");
	
	if(num>sen->queue_count){
		printf("Queue Resest more then vaiable %d %d \n",sen->queue_count,num);
		num=sen->queue_count;
	}

	printf("Writing to %s %d samples \n",buff,num);
	//We run for the end of the queue to the start
	// we start at  end-num
	//idx=sen->queue_write - (num+1 ); // Two samples more to be sure ...
	idx=sen->queue_write - (num ); // Two samples more to be sure ...
	if(idx<0)idx+=(Q_SIZE);
	const int idx_start=idx;
	
	// Compute mean first
	for(i=0;i<num;i++){
		idx=(idx+1)%Q_SIZE;
		mean_x+=sen->x[idx];
		mean_y+=sen->y[idx];
		mean_z+=sen->z[idx];
	}
	mean_x=0.;//mean_x/(float)num;
	mean_y=0.;//mean_y/(float)num;
	mean_z=0.;//mean_z/(float)num;

	// write out 
	idx=idx_start;
	printf("Start Index is %d \n",idx);
	for(i=0;i<num;i++){
		idx=(idx+1)%Q_SIZE;
		fprintf(out_file,"%.4f  %.4f %.4f\n",
			sen->x[idx]-mean_x,sen->y[idx]-mean_y,sen->z[idx]-mean_z);
	}
	fclose(out_file);
}

void write_disk_all(int r_nr, int num)
{
	int i;
	printf("Saving all sensors index %d \n",r_nr);
	for(i=0;i<NUM_SENSOR;i++){
		if(sensors[i]!=0) write_disk(sensors[i],1,5000);
	}
}


/*-------------------------------------- */

//SOCKED THINGS

int sockfd; /// The socket we talk to

/* 
 * error - wrapper for perror
 */

void error(char *msg)
{
    perror(msg);
    exit(0);
}
/*-------------------------------------- */
// Data queue per sensor
// These are designed to overwirte if more data comming
// and flush to disk

void add_value(sensor* sen,float x,float y, float z)
{

	sen->x[sen->queue_write]=x;
	sen->y[sen->queue_write]=y;
	sen->z[sen->queue_write]=z;

	pthread_mutex_lock(&(sen->w_lock)); //CRITICAL SECTION 
	sen->queue_write=(sen->queue_write+1)%Q_SIZE;
	sen->queue_count++;
	if(sen->queue_count>=Q_SIZE)sen->queue_count=Q_SIZE;
	pthread_mutex_unlock(&(sen->w_lock)); 

	// IF WE ARE TRIGGERED WE WAIT UNTIL WE HAVE THE RIGHT AMOUT OF DATA AND SAVE
        if(trigger_save_file==1){
	}
                        
}

/*-------------------------------------- */


#define BYTE unsigned char
#define WORD short


WORD MAKE_WORD( const BYTE Byte_hi, const BYTE Byte_lo)
{
    return   (( Byte_hi << 8  ) | (Byte_lo & 0x00FF) );
}



float convert_val(unsigned char ch1, unsigned char ch2)
{
	float val;
	//int ival=(((unsigned int)ch1)*256+((unsigned int)ch2));
	int ival=(((unsigned int)ch1)*256+((unsigned int)ch2));
	//int ival= (int)(ch1)<<8|  (int)ch2;
	//unsigned int ival= ((unsigned int)(ch1)<<8)| ((unsigned int)ch2);

	//val=((float)(ival)); // Just Raw
	//val=((float)(ival))* 1./1024 - 2. ; // +-2g so  4./4096  -2
	//We fo 12 bit now
	//val=((float)(ival))*1./4096    ; // +-2g so  4./4096  -2
	if(g_range==0)return((float)MAKE_WORD(ch1,ch2) * 1/(4.*4096.));
	if(g_range==1)return((float)MAKE_WORD(ch1,ch2) * 1/(2.*4096.));
	if(g_range==2)return((float)MAKE_WORD(ch1,ch2) * 1/(16.*4096.));
	return((float)MAKE_WORD(ch1,ch2) * 1/(32.*4096.));

//	printf("[%3u,%3u] -> %f \n ",(unsigned char)ch1,(unsigned char)ch2,val);

	return(val);
}
/*-------------------------------------- */
//Socket reader for sensor



// Blocking read on socket with buffer ...
unsigned char sock_read(sensor* sen)
{

 //Check we we have to read more
 if((sen->buf_pos>=sen->buf_num) || (sen->buf_num==0)){
		//printf("READ ");
               sen->buf_num=read(sen->fd,sen->buf,BUF_LEN);
	       //printf("GOT %d \n ",sen->buf_num);
               sen->buf_pos=0;
 }
 const int idx=sen->buf_pos;
 sen->buf_pos++;

 return(sen->buf[idx]);

}


int sock_avaiable(sensor* sen)
{
       return(sen->buf_num-sen->buf_pos-1);
}
/*-------------------------------------- */
// Trigger watcher THREAD

/*-------------------------------------- */
// READER THREAD

#include <pthread.h>



static int running=1;


#define BUFSIZE 8000


// ARG  is the sensor structure
void* value_reader(void* arg)
{
 int k;
 sensor* sen=(sensor*)arg;
 printf("READER STARTED for sensor %d \n",sen->id);
 int sockfd= sen->fd;
 unsigned char ch1;
 unsigned char ch2;


 unsigned char next_id=1; //Packet conter starts at 1 !
 int counter=0;

	ch1=sock_read(sen);
	ch2=sock_read(sen);

 while(running){
        // Critical Section
	float x,y,z;

	while(!((ch1==(unsigned char)255)&&(ch2==(unsigned char)255))){
		ch1=ch2;
		ch2=sock_read(sen);
		printf("TICK \n");
	}

        //printf("->%3u,%3u ",(unsigned char)ch1,(unsigned char)ch2);
	printf("LOOP %d \n",counter);counter++;

	if((ch1==(unsigned char)255)&&(ch2==(unsigned char)255)){ // WE READ A DATA SET

		//if the next two are also 255 then this was the last !!
		ch1=sock_read(sen);
		ch2=sock_read(sen);
		//SHOULD NOT HAPPEN
		if((ch1==(unsigned char)255)&&(ch2==(unsigned char)255)){
			//AT THE END
			printf("END OF STRAMING \n");
			return(0);
		}
		

		for(k=0;k< 200;k++){ // 1200 bytes, 200 *2 bytes  *3  axis
			x =convert_val(ch1,ch2); // We pre-read to decide
			y =convert_val(sock_read(sen),sock_read(sen));
			z =convert_val(sock_read(sen),sock_read(sen));
		//	if(k%20==0)printf("%d %f %f %f \n",k,x,y,z);
				
			//PUT DATA INTO BUFFER
			add_value(sen,x,y,z);
			//TRIGGER 
        		if(sen->id==trig_sen_id){
			         check_trigger(x,y,z);
				 trigger_queue_pos= 12324; // queue pos

			}


			ch1=sock_read(sen);
			ch2=sock_read(sen);
		}
		// THESE SENROS SEND 6 TOO MANY !!!!
		ch1=sock_read(sen);
                ch2=sock_read(sen);

                ch1=sock_read(sen);
                ch2=sock_read(sen);


                ch1=sock_read(sen);
                ch2=sock_read(sen);


		// We did read 2 modre !
		//NOW comes THE RUNNING COUNTER !
		if(ch1!=ch2)printf("Checksum ERROR %3u to %3u \n", ch1,ch2);

		//printf("Packet counter %3u %3u \n",(unsigned char)ch1,(unsigned char)next_id);
		if(ch1!=next_id)printf("Lost Packet \n");
		next_id=(unsigned char)(ch1+1)%128;

		//OK we got another pack of 200, signal to the consumer
        	pthread_cond_signal(&(sen->dava_rdy));
/*

		
		ch1=sock_read(sen);
		ch2=sock_read(sen);

		ch1=sock_read(sen);
		ch2=sock_read(sen);


		ch1=sock_read(sen);
		ch2=sock_read(sen);

		if(ch1!=ch2)printf("Checksum ERROR @@@@@  %3u to %3u \n", ch1,ch2);
		printf("Packet counter &&& %3u %3u \n",(unsigned char)ch1,(unsigned char)ch2);
*/

		ch1=sock_read(sen);
		ch2=sock_read(sen);

	}else{
		printf(" READER ERROR \n");
	}
 }
 printf(" Reader loop stopped \n");
 return(0);
}

/*---------------------------------------------*/
// This is the data processor thread
// We could run sseveral of thee (one per sensor, or just one
// if waits for the condition   sen->dava_rdy (per sensor)
// and if read processes 200 samples
// i use the mutes in there too (even it is not stricktly needed !
//  So it will be sleeping while new data is added to the queue


void* data_processor(void* args)
{
	sensor* sen=sensors[0]; // Do it over one sensors ...
 	while(running){
		pthread_mutex_lock(&(sen->d_lock));        
		pthread_cond_wait(&(sen->dava_rdy),&(sen->d_lock));
		printf("Data Reader got data ready signal %d  \n",sen->queue_count);
		pthread_mutex_unlock(&(sen->d_lock));

	}
	return(0);
}

/*---------------------------------------------*/

// read the small responses to commands
#define SM_BUFF  62
void response_reader(sensor* sen,unsigned char cmd,int num_res)
{
	unsigned char buf[SM_BUFF];
	int n,i;
	n = read(sen->fd, buf , SM_BUFF );
	printf("RESPONSE S%d = ",sen->id);
	for(i=0;i<n;i++){
		printf("%3u, ",(unsigned char)buf[i]);
	}
	printf("\n");
}

// THe protocol seems not proper, so after soppting streaming I use this to ust read the rest of
// the stream until we back in a normal state
void sock_clear(sensor* sen)
{
	unsigned char buf[4096];
	int n;
	ioctl(sockfd, FIONREAD, &n);
	printf("Clean socket %d \n",sen->id);
	while(n>0){
		n = read(sen->fd, buf , 4096);
		printf("%d ",n);
		ioctl(sockfd, FIONREAD, &n);
	}
	printf("\n");
}




int setup_connection(const char* hostname)
{

    int  portno, n;
    int sockfd;
    struct sockaddr_in serveraddr;
    struct hostent *server;


    portno=25001; // HARD CODED

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");

    /* gethostbyname: get the server's DNS entry */
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        exit(0);
    }

    /* build the server's Internet address */
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serveraddr.sin_addr.s_addr, server->h_length);
    serveraddr.sin_port = htons(portno);

    /* connect: create a connection with the server */
    if (connect(sockfd,(struct sockaddr *)  &serveraddr, sizeof(serveraddr)) < 0){
      error("ERROR connecting"); 
      return(0);
    }

    printf("CONENCTED %s  \n",hostname);

    return(sockfd);

}


//Make sensors, connet them and start readers


void setup_sensors(void)
{
	int i;
	for(i=0;i<NUM_SENSOR;i++)sensors[i]=0;

	printf("Sensor CONNECTING \n");

	sensors[0]=make_sensor(1);
	sensors[0]->fd=setup_connection("172.30.150.211");

	// ADD MORE 
	
	//sensors[1]=make_sensor(2);
	//sensors[1]->fd=setup_connection("172.30.100.212");
	/*
	
	sensors[2]=make_sensor(3);
	sensors[2]->fd=setup_connection("172.30.100.213");

	sensors[3]=make_sensor(4);
	sensors[3]->fd=setup_connection("172.30.100.214");
	

	*/

};

//SEND ALL SENSORS A COMMAND
// READY is ^
// START is   { STOP IS }
void send_all(char cmd)
{
	int res,i;
	char buff[16];
	buff[0]=cmd;
	for(i=0;i<NUM_SENSOR;i++){
		if(sensors[i]!=0) res = write(sensors[i]->fd,buff,1 );
	}
	for(i=0;i<NUM_SENSOR;i++){
		if(sensors[i]!=0) response_reader(sensors[i],cmd,2);
	}

}


/*------------------------------------*/
// NOT DONE/TESTED YET
// Send a ?XX (XX depends on g) read respnse (24 bytes) ...

void get_sensor_calibration(sensor* sens)
{
	int i,res;
	char buff[25];
        buff[0]='?'; buff[1]=0x01;
//	buff[1]=0xa8; // 2G
	res = write(sens->fd,buff,2 );
	//response_reader(sens,buff[0],25);
	//response_reader(sens,buff[0],25);

	// WE WILL GET 25 NUMBERS IN RESPINSE
	i=0;
	while(i<25){
		i+=read(sens->fd,&(buff[i]),25);

        	//buff[i]=sock_read(sens);
        }
	for(i=0;i<25;i++)printf("%3u, ",(unsigned char)buff[i]);
	printf("\n");

	//First tow are repettion
	//buff[0]=? buff[1]=g_sel;
	float* val;
	for(i=0;i<6;i++){	
		val=(float*)&(buff[(4*i)+2]);
		printf("Val %d  %f \n",i, *val);
	}

	printf("Calibrated \n");

	//response_reader(sens,buff[0],25);

	
}


/*------------------------------------*/
// Set up threads to read data form each sensor, send { to start straming
// if done send } to stop and wait for the last packed

pthread_t sensor_thread[NUM_SENSOR];
pthread_t processor_thread;

void stream_data(void)
{
	char buf[16];
	int i,n;
	printf("Sart Streaming \n");
	buf[0]='{';
	running=1;
	//START READER FOR ALL SENSORS
	for(i=0;i<NUM_SENSOR;i++){
                if(sensors[i]!=0){
        		pthread_create(&(sensor_thread[i]), NULL, &value_reader,(void*)sensors[i]);
			// Tell them to start
			n = write(sensors[i]->fd, buf, 1);
		}
	}
	// Start the data processor
	// This is to add mor functionality, realtime  processing
        pthread_create(&processor_thread, NULL, &data_processor,NULL);

	printf("Readers started \n");

	int rec_num=0; // The index of the recording for this session

	

	buf[0]=0;
	while(buf[0]!='q'){
		printf("CMD ( a- arm r - record, q - stop)\n");
/*
		bzero(buf, 16);
		fgets(buf, 16, stdin);
*/
		    buf[0] = fgetc(stdin);

     		if(buf[0]=='a'){ //Arm Trigger
               		 init_trigger(1,2.5);
                	continue;
        	}


		if(buf[0]=='r'){ // Record the data
			write_disk_all(rec_num,5000);
			rec_num++;
		}
	}

	//STOP all sensor  streaming
	buf[0]='}';
	for(i=0;i<NUM_SENSOR;i++){
                if(sensors[i]!=0) n = write(sensors[i]->fd, buf, 1);
	}
	running=0; //STOP READER THREAD !!
	printf("Waiting for Join (ret) \n");
	fgets(buf, 16, stdin);


	// Write all to disk
	write_disk_all(rec_num,5000);

	// JOIN the threads back
	void* ret_val;
	for(i=0;i<NUM_SENSOR;i++){
                if(sensors[i]!=0) pthread_join(sensor_thread[i],ret_val);
        }

	// Only if the hage the processor running ! (don't delete)
	//pthread_join(processor_thread, ret_val);


	printf("STREAMING DONE reader joined \n");
	//  Clear the sockets (see my commen in the the protocol 
	// Thus should not be, but sensors do that ...
        for(i=0;i<NUM_SENSOR;i++){
                if(sensors[i]!=0) sock_clear(sensors[i]);
        }
}

/*---------------------------------------------------------------*/
// Interactive with one sensor over keyboard

#define SEND_BU 256


void sen_interactive(sensor* sen)
{
	int n;
	char buf[SEND_BU];
	sockfd=sen->fd;
  printf("READY \n");

  int val=1;
  while(val==1){
        // Comands are  ^  { .... 
        printf("msg: ");
        bzero(buf, SEND_BU);
        fgets(buf, SEND_BU, stdin);

        if(buf[0]=='q'){
                        break;
                        running=0;
        };
        if(buf[0]=='c'){ //Calibration
		get_sensor_calibration(sensors[0]);
		continue;
        }


        if(buf[0]=='{'){ // START RECORING 
                printf("Start record \n");
                running=1;
                stream_data();
        }else{

                // send the message to the server 
                n = write(sockfd, buf, strlen(buf)-1);
                if (n < 0) error("ERROR writing to socket");
                response_reader(sensors[0],buf[0],-1);// print  respoinse
        }
    };

}
/*------------------------------------*/


int main(int argc, char **argv)
{

    	unsigned char buf[SEND_BU];

  	setup_sensors();

	// GET ALL READY
	send_all('^');

	// MOVE THIS TO THE SETUP !!
	// Select G range
	/* Manual says
	a. 0xa8-2g
	b. 0xa9-4g
	c. 0xaa-8g
	d. 0xab-16g
	*/
	//send_all(0xA8);
	g_range=1;
	send_all(0xA9);
	

  // These sensors do not change sample rate ?

	// set a trigger int sen id, float thresh
	init_trigger(1,2.5);

	


  printf("SETUP DONE \n");
   // Get calibration
  //get_sensor_calibration(sensors[0]);
  sen_interactive(sensors[0]);


    return 0;
}
