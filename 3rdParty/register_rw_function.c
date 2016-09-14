#include <stdio.h>
#include <fcntl.h>
#include <linux/videodev2.h>
                      
/*------------------------------------------------------------------------*/
/*      ImageSensor Register Read/Write Function                          */
/*------------------------------------------------------------------------*/
int m05vu_reg_write(int, unsigned char, unsigned char );
int m05vu_reg_read(int, unsigned char, unsigned char*);


/*------------------------------------------------------------------------*/
/*   Test main 															  */
/*------------------------------------------------------------------------*/
int main(int argc,char *argv[])
{
  int i;
  int rtn;
  char command[64];

  if( argc != 2){ 
    fprintf(stdout,"argument error:\n");
  }

  /* camera open */
  int fd = open(argv[1], O_RDWR | O_NONBLOCK);
  if( fd == -1 ){  
    fprintf(stdout,"camera can't open:%s\n",argv[1]);
  }

  /* menu */
  printf("\n--- KBCR-M05VU Register Read/Write Test--- \n\n");
  printf(" r(R) adr        : Read Register  \n");
  printf(" w(W) adr data   : Write Register \n");
  printf(" q(Q)            : Quit\n");


 char com;
 int  p1=-1;
 int  p2=-1;
 int  p3=-1;
 int  num;

 unsigned char reg_val;
 unsigned char reg_adr;

LOOP: 
  printf("\ninput > ");
 fgets(command,sizeof(command),stdin);
 num = sscanf(command,"%c %x %x %x",&com,&p1,&p2,&p3); 
   
  switch( com){

   case 'r':
   case 'R':
     if(num == 2){  

 	    printf("Read Register:0x%02X", p1);

        reg_adr = (unsigned char)p1;
        /* Register Read */
        if( m05vu_reg_read(fd, reg_adr, &reg_val) != 0 ) {
           printf("\n NG: Register Read Error! \n");
        }
        else{ printf("  0x%02X\nOK\n", reg_val); }

     }
     else {
	   printf("Bad command input!\n");
     }
     break;
 
   case 'w':
   case 'W':
     if(num == 3){
 	    printf("Write Register:0x%02X 0x%02X", p1, p2);

        reg_adr = (unsigned char)p1;
        reg_val = (unsigned char)p2;
        /* Register Write */
        if( m05vu_reg_write(fd, reg_adr, reg_val) != 0 ) {
           printf("\n NG: Register Write Error! \n");
        }
        else{  printf("\nOK\n"); }
     }
     else {
	   printf("Bad command input!\n");
     }
     break;
 
   case 'q':
   case 'Q':
	 close(fd);
     return;

  }

  goto LOOP;

}


/* --------------------------------------------------------------------------*/
/*      ImageSensor Register Read/Write Function                             */
/* --------------------------------------------------------------------------*/
/*
 * Command ID
 */
#define COM_TYPE_COM_STT		(0xFF)           /* Command ID [START]      */
#define COM_TYPE_COM_END		(0xFE)           /* Command ID [END]        */
#define COM_TYPE_REG_WR		    (0xFA)           /* Command ID [REG WRITE]  */
#define COM_TYPE_REG_RD		    (0xF9)           /* Command ID [REG READ ]  */

/*
 * (Sub Routine) IRIS contorol
 */
int myIrisGetControl(int);                       /* IRIS property Get       */
int myIrisSetControl(int, unsigned char);        /* IRIS property Set       */

/* --------------------------------------------------------------------------
 *  Register Write Function
 *
 * hcam		    handle obtained from V4L2 open()
 * addr 	    register address
 * data			register data
 *
 * return
 * 		     0 (on success)
 * 		    -1 (if an error has occurred)
---------------------------------------------------------------------------- */

int m05vu_reg_write(int hcam, unsigned char addr, unsigned char data )
{
  int rtn;
  
  /* Command Start */
  if ((rtn = myIrisGetControl(hcam                     )) < 0) { goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_STT) )) < 0) { goto error_end;}

  /* Regiter Write */
  if ((rtn = myIrisGetControl(hcam                     )) < 0) { goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (COM_TYPE_REG_WR)  )) < 0) { goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (addr)             )) < 0) { goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (data)             )) < 0) { goto error_end;}

  /* Command End */
  if ((rtn = myIrisGetControl(hcam                     )) < 0) { goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_END) )) < 0) { goto error_end;}

  return 0;   /* Success */

error_end:
  return rtn; /* Error */
}


/* --------------------------------------------------------------------------
 * Register Read Function
 *
 * hcam		    handle obtained from V4L2 open()
 * addr 	    register address
 * data			register data (*pointer)
 *
 * return
 * 		     0 (on success)
 * 		    -1 (if an error has occurred)
---------------------------------------------------------------------------- */
int m05vu_reg_read(int hcam, unsigned char addr, unsigned char* data)
{

  int rtn;
  int reg_data;
 
  /* Command Start */
  if ((rtn = myIrisGetControl(hcam                     )) < 0){ goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_STT) )) < 0){ goto error_end;}

  /* Regiter Read */
  if ((rtn = myIrisGetControl(hcam                     )) < 0){ goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (COM_TYPE_REG_RD)  )) < 0){ goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (addr)             )) < 0){ goto error_end;}

  if ((reg_data = myIrisGetControl(hcam            )) < 0){ goto error_end;}
  else{
      *data = (unsigned char)reg_data; /* register value */
  }
   
  /* Command End */
  if ((rtn = myIrisGetControl(hcam                     )) < 0){ goto error_end;}
  if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_END) )) < 0){ goto error_end;}

  return 0;   /* Success */

error_end:
  return rtn; /* Error */

}


/*
 * (Sub Routine) IRIS contorol
 */

int myIrisGetControl(int hcam )
{
    struct v4l2_control control_s;
    int err;

    control_s.id = V4L2_CID_IRIS_ABSOLUTE;

    if ((err = ioctl( hcam, VIDIOC_G_CTRL, &control_s)) < 0) {
  	  return -1;
    }

    return control_s.value;
}

int myIrisSetControl(int hcam, unsigned char value)
{
    struct v4l2_control control_s;
    int err;

	control_s.id = V4L2_CID_IRIS_ABSOLUTE;
	control_s.value = value;

	if ((err = ioctl( hcam, VIDIOC_S_CTRL, &control_s)) < 0) {
	    return -1;
	}

    return 0;
}

