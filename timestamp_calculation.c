//Program for data acquisition

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <omp.h>

void acq_timestamp(unsigned long timestamp[],int n_events,unsigned long *time_array, int *pattern_array);
void get_histo(unsigned long time_array[20],int pattern_array[20],int n_events, int *histogram,int *cnt2,int *cnt4);


int serial_port;

// Create new termios struct
struct termios tty;


int init_device(const char port_name[]){
  
  //Initialises the device
  serial_port = open(port_name, O_RDWR);

  // Check for errors
  if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
  }

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr() must
  // have been initialized with a call to tcgetattr() overwise behaviour is undefined
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  struct termios {
  	tcflag_t c_iflag;		/* input mode flags */
  	tcflag_t c_oflag;		/* output mode flags */
  	tcflag_t c_cflag;		/* control mode flags */
  	tcflag_t c_lflag;		/* local mode flags */
  	cc_t c_line;			/* line discipline */
  	cc_t c_cc[NCCS];		/* control characters */
  };

  tty.c_cflag &= ~PARENB; // Disables parity

  tty.c_cflag &= ~CSTOPB; //Only use one stop bit

  tty.c_cflag &= ~CSIZE; // Clear all the size bits
  tty.c_cflag |= CS8; // 8 bits per byte

  tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control

  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

  tty.c_lflag &= ~ICANON; //Disables canonical mode

  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control

  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received byte

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;    // Wait 1 s,return when any data is received
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  //Activate timestamp mode
  char msg[] = "*RST;TTL;TIME75;TIMESTAMP;";
  write(serial_port, msg, strlen(msg));
  
  return 0;
}

int close_device(){
  close(serial_port); //Closes the device
  return 0;
}

int * stamp(double t_collect,int *hist_res) {
  
  for(int i = 0; i<23; i++){
    hist_res[i] = 0;
  }
  
  double elapsed = 0.0; 

  //Start measuring time
  struct timespec begin, end;
  clock_gettime(CLOCK_MONOTONIC,&begin);

  //Contiously collects data for t_collect time
  while(elapsed < t_collect){
    //Ask for data
    char msg[] = "counts?;";
    write(serial_port, msg, strlen(msg));

    int n_events_tot = 0;
    unsigned long* read_buf;
    read_buf = (unsigned long int*)calloc(1,sizeof(unsigned long));
    if(read_buf == NULL){ //Error management
      continue;
    }

    while(1){ //Reads until the buffer is empty (less than 4 bytes, since 4 bytes is 1 event)
      
      int n_read;
      ioctl(serial_port, FIONREAD, &n_read); //Check how much data is available to be read
      
      if(n_read < 4){
        break;
      }
      
      int n_events = n_read/4;
      n_events_tot += n_events;
      unsigned long temp_read_buf[n_events];
      int bytes_read = read(serial_port, &temp_read_buf, 4*n_events);
    
      read_buf = realloc(read_buf,n_events_tot * sizeof(unsigned long)); //Changes the size of read_buf as more data comes in
      if(read_buf == NULL){ //Error management
        n_events_tot = 0;
        break;
      }

      
      for(int i = 0; i < n_events;i++){
        read_buf[i + n_events_tot - n_events] = temp_read_buf[i];
      }

    }
    
    if(n_events_tot == 0){
      
      free(read_buf); //Frees the read buffer

      clock_gettime(CLOCK_MONOTONIC,&end);
      long seconds = end.tv_sec - begin.tv_sec;
      long nanoseconds = end.tv_nsec - begin.tv_nsec;
      elapsed = seconds + nanoseconds*1e-9; //Get the elapsed time
      
      usleep(0.001); //If no data was received, wait 1 nanosecond before trying again
      continue;
    }
   
    
    //Multithreading with 4 threads
    #pragma omp parallel num_threads(4)
    {
      #pragma omp for 
      for(int j = 0;j<4;j++){ //Splitting read_buf into 4 seperate arrays
        int n_split;
        if(j != 3){
          n_split = n_events_tot/4;
        }else{
          n_split = n_events_tot - 3*(n_events_tot/4);
        }

        unsigned long split_buf[n_split];
        for(int i = 0;i<n_split;i++){
          split_buf[i] = read_buf[j*n_events_tot/4 + i];
        }
        


        //Adds the time for events aswell as the pattern (last 5 numbers) into arrays
        int pattern_array[n_split];
        unsigned long time_array[n_split];
        acq_timestamp(split_buf, n_split, time_array, pattern_array);

        //Collects a histogram
        int cnt2;
        int cnt4;
        int histogram[21];

        get_histo(time_array, pattern_array, n_split, histogram, &cnt2, &cnt4);

        for(int i = 0; i<21; i++){
          hist_res[i] += histogram[i]; //Adds to the total histogram
        }

        hist_res[21] += cnt2;
        hist_res[22] += cnt4;

      }
    }
    
    free(read_buf); //Frees the read buffer
    
    clock_gettime(CLOCK_MONOTONIC,&end);
    long seconds = end.tv_sec - begin.tv_sec;
    long nanoseconds = end.tv_nsec - begin.tv_nsec;
    elapsed = seconds + nanoseconds*1e-9; //Get the elapsed time
    
    //Clears the buffer
    int n_clear;
    ioctl(serial_port, FIONREAD, &n_clear);
    read(serial_port, NULL, n_clear);
  
  }

  return hist_res; // success

}

void acq_timestamp(unsigned long timestamp[], int n_events,  unsigned long *time_array, int *pattern_array){
  //Returns timestamp and pattern

  int binary_array[n_events][32];
  for(int j=0;j<n_events;j++){
    //Converts timestamp integer to binary format. The binary number is taken in reverse
    int single_timestamp = timestamp[j];
    for(int i=0;i<32;i++){
      binary_array[j][i] = single_timestamp%2;
      single_timestamp /= 2;
    }
  }

  //Converts time in nanoseconds and pattern to integer.
  for(int j=0;j<n_events;j++){
    long int ret_time = 0;
    for(int i=5;i<32;i++){
      //Converts first 27 binary numbers to an integer
      ret_time += binary_array[j][i]*pow(2,i-5)*2;
    }
    time_array[j] = ret_time;
    int pattern = 0;
    for(int i=0;i<5;i++){
      //Adds pattern as an integer (instead of binary)
      pattern += binary_array[j][i]*pow(2,i);
    }
    pattern_array[j] = pattern;
  }

}

void get_histo(unsigned long time_array[],int pattern_array[],int n_events, int *histogram,int *cnt2,int *cnt4){
  //Calculates and returns histogram
  
  memset(histogram,0, 21*sizeof(int));

  int temp_cnt2;
  int temp_cnt4;
  if(pattern_array[0] == 8){
    temp_cnt2 = 0; //#Event in channel 2
    temp_cnt4 = 1; //#Event in channel 4
  }else if(pattern_array[0] == 2){
    temp_cnt2 = 1; //#Event in channel 2
    temp_cnt4 = 0; //#Event in channel 4
  }else if(pattern_array[0] == 10){
    temp_cnt2 = 1;
    temp_cnt4 = 1;
    histogram[10]++;
  }else{
    temp_cnt2 = 0;
    temp_cnt4 = 0;
  }
  for(int j=1;j<n_events;j++){
    int prev_pattern = pattern_array[j-1];
    long int delay = time_array[j] - time_array[j-1];

    //Counts rate of events
    if(pattern_array[j] == 8){
      temp_cnt4++;
    }else if(pattern_array[j] == 2){
      temp_cnt2++;
    }else if(pattern_array[j] == 10){
      //Event in both channels
      histogram[10]++;
      temp_cnt2++;
      temp_cnt4++;
      continue;
    }else{
      continue;
    }
    if(delay<0){
      continue;
    }
    //Events larger than 20 ns between are ignored
    if(delay<22){
      //This assumes positive delay if event at channel 4 happens after channel 2
      if(pattern_array[j] == 8 && prev_pattern == 2){
        histogram[10+delay/2]++;
        }
      else if(pattern_array[j] == 2 && prev_pattern == 8){
        histogram[10-delay/2]++;
        }
      }
    }
    
    *cnt2 = temp_cnt2;
    *cnt4 = temp_cnt4;
}
