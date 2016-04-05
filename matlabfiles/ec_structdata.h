/* module input/output structure config for SOEM Master 
// (c) 2008 R.Solberg IME Technologies
*/
/*EL1xxx------------------------------*/
typedef struct PACKED
{
	uint8	   inbits;
} in_EL1008t;	

typedef struct PACKED
{
	uint8	   inbits;
} in_EL1014t;	

typedef struct PACKED
{
	uint8	   inbits;
} in_EL1018t;	


/*EL2xxx------------------------------*/
typedef struct PACKED
{
	uint8	   outbits;
} out_EL2004t;	

typedef struct PACKED
{
	uint8	   outbits;
} out_EL2008t;	

typedef struct PACKED
{
    uint16      pwm1;
    uint16      pwm2;
} out_EL2502t;


/*EL3xxx------------------------------*/
typedef struct PACKED
{
	int16	   status1;
	int16	   invalue1;
	int16	   status2;
	int16	   invalue2;
	int16	   status3;
	int16	   invalue3;
	int16	   status4;
	int16	   invalue4;
	int16	   status5;
	int16	   invalue5;
	int16	   status6;
	int16	   invalue6;
	int16	   status7;
	int16	   invalue7;
	int16	   status8;
	int16	   invalue8;

} in_EL3008t;

typedef struct PACKED
{
	uint8	   status1;
	int16	   invalue1;
	uint8	   status2;
	int16	   invalue2;
} in_EL3102t;	


/*EL4xxx------------------------------*/
typedef struct PACKED
{
	int16	   outvalue1;
	int16	   outvalue2;
} out_EL4132t;

typedef struct PACKED
{
	int16	   outvalue1;
	int16	   outvalue2;
    int16	   outvalue3;
    int16	   outvalue4;
    int16	   outvalue5;
    int16	   outvalue6;
    int16	   outvalue7;
    int16	   outvalue8;
} out_EL4038t;

/*EL5xxx------------------------------*/
typedef struct PACKED
{
	uint8	   control;
	int16	   outvalue;
} out_EL5101t;	

typedef struct PACKED
{
	uint8	   status;
	int16	   invalue;
	int16	   latch;
} in_EL5101t;

typedef struct PACKED
{
	uint16	   control0;
	int32	   outvalue0;
	uint16	   control1;
	int32	   outvalue1;
} out_EL5152t;

typedef struct PACKED
{
	uint16	   status0;
	int32	   invalue0;
	int32	   period0;
	uint16	   status1;
	int32	   invalue1;
	int32	   period1;
} in_EL5152t;

/*TU0001 -- intelligent slave for Robocup, contains AD, SerialCAM interface, shootercontrol, acceleration */
typedef struct PACKED
{
	uint16	   CAM1control;
	uint32	   CAM1data;
	uint8	   SHTcontrol;	
	uint8	   SHTdutycycle;
	uint8	   SHTpulselength;
	uint8	   SHTencodermax;
} out_TU0001t;	

typedef struct PACKED
{ 
	uint8         status;
	uint8         counter;
	uint8         diginput;
	int16         analog[8];
	uint16        CAM1status;
	int16         CAM1posX;        
	int16         CAM1posY;        
	int16         CAM1posZ;        
	int16         CAM1velX;        
	int16         CAM1velY;        
	int16         CAM1velZ;        
	uint32        CAM1timestamp;
	uint8         SHTstatus;
	int16         SHTvalue;
        int16         AccelerationX;
        int16         AccelerationY;
        int16         AccelerationZ;
} in_TU0001t;	

/*TU_ES_030 -- intelligent slave for SERGIO Arm */
typedef struct PACKED
{
	uint8       port;
	int16       m_1;
    int16       m_2;
    int16       m_3;
    int16       aout_1;
    int16       aout_2;
} out_TU_ES_030t;	

typedef struct PACKED
{ 
    uint8       port;
    uint16      encoder_1;
    uint16      encoder_2;
    uint16      encoder_3;
    int16       current_1;
    int16       current_2;
    int16       current_3;
    uint16      caliper_1;
    uint16      caliper_2; 
    uint16      force_1;
    uint16      force_2;
    uint16      force_3;
    uint16      pos_1;
    uint16      pos_2;
    uint16      pos_3;
    uint16      spare_ai_1;
    uint16      spare_ai_2;
    uint16      time_stamp;            
} in_TU_ES_030t;

/*TU_ES_030V2 -- intelligent slave for SERGIO Arm, update Ketels */
typedef struct PACKED
{
	uint8       mcom1;
	int16       setpoint1;
    int16       ff1;
	uint8       mcom2;
	int16       setpoint2;
    int16       ff2;
    uint8       mcom3;
	int16       setpoint3;
    int16       ff3;
    uint8       digital;
    int16       aout1;
    int16       aout2;
} out_TU_ES_030v2t;	

typedef struct PACKED
{ 
    uint8       mstate1;
    uint32      count1;
    uint32      timestamp1;
    int16       velocity1;
    int16       current1;
    uint8       mstate2;
    uint32      count2;
    uint32      timestamp2;
    int16       velocity2;
    int16       current2;
    uint8       mstate3;
    uint32      count3;
    uint32      timestamp3;
    int16       velocity3;
    int16       current3;
    uint8       digital;
    uint16      caliper1;
    uint16      caliper2;
    uint16      force1;
    uint16      force2;
    uint16      force3;
    uint16      pos1;
    uint16      pos2;
    uint16      pos3;
    uint16      analog1;
    uint16      analog2;
    uint16      linevoltage;
    uint16      ectime;         
} in_TU_ES_030v2t;	

//couple structures to pointerarrays---------------------*/	

in_EL1008t		*in_EL1008_[MAX_NUMBER_OF_SLAVES];
in_EL1014t		*in_EL1014_[MAX_NUMBER_OF_SLAVES];
in_EL1018t		*in_EL1018_[MAX_NUMBER_OF_SLAVES];

out_EL2004t		*out_EL2004_[MAX_NUMBER_OF_SLAVES];
out_EL2008t		*out_EL2008_[MAX_NUMBER_OF_SLAVES];
out_EL2502t		*out_EL2502_[MAX_NUMBER_OF_SLAVES];

in_EL3008t		*in_EL3008_[MAX_NUMBER_OF_SLAVES];
in_EL3102t		*in_EL3102_[MAX_NUMBER_OF_SLAVES];

out_EL4132t		*out_EL4132_[MAX_NUMBER_OF_SLAVES];
out_EL4038t		*out_EL4038_[MAX_NUMBER_OF_SLAVES];

out_EL5101t		*out_EL5101_[MAX_NUMBER_OF_SLAVES];
in_EL5101t		*in_EL5101_[MAX_NUMBER_OF_SLAVES];
out_EL5152t		*out_EL5152_[MAX_NUMBER_OF_SLAVES];
in_EL5152t		*in_EL5152_[MAX_NUMBER_OF_SLAVES];

/* maximum of 2 TU0001 slaves */
out_TU0001t		*out_TU0001_[2];
in_TU0001t		*in_TU0001_[2];

out_TU_ES_030t		*out_TU_ES_030_[6];
in_TU_ES_030t		*in_TU_ES_030_[6];

out_TU_ES_030v2t    *out_TU_ES_030v2_[6];
in_TU_ES_030v2t		*in_TU_ES_030v2_[6];
