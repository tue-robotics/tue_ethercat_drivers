
int ec_EL4132_dac_write_chan(double, int, int);
int ec_EL4038_dac_write_chan(double, int, int);
int ec_EL2004_do_write_chan(double, int, int);
int ec_EL2008_do_write_chan(double, int, int);
int ec_EL2502_pwm_write_chan(double, int, int);
int ec_EL5101_enc_read_chan(double *, int);
int ec_EL5152_enc_read_chan(double *, int);
int ec_EL3008_adc_read_chan(double *, int, int);
int ec_EL3102_adc_read_chan(double *, int, int);
int ec_EL1014_di_read_chan(double *, int, int);
int ec_EL1008_di_read_chan(double *, int, int);
int ec_EL1018_di_read_chan(double *, int, int);
int ec_TU0001_combi_write_chan(double, int, int);
int ec_TU0001_combi_read_chan(double *, int, int);
int ec_TU_ES_030_read_chan(double *, int, int);
int ec_TU_ES_030_write_chan(double, int, int);
int ec_TU_ES_030v2_read_chan(double *, int, int);
int ec_TU_ES_030v2_write_chan(double, int, int);

int ec_io(void);
int ec_i(void);
int ec_o(void);
int ec_initial(char*);
int ec_scan(void);
int ec_exit(void);

/* Maximum number of slaves in one stack */
#define MAX_NUMBER_OF_SLAVES  25

/* define a struct for encoder data of encoder interfaces */
#define MAX_NUMBER_OF_ENC_CHANNELS_PER_SLAVE 2

typedef struct
{
	int count0[MAX_NUMBER_OF_ENC_CHANNELS_PER_SLAVE];
	int initialized[MAX_NUMBER_OF_ENC_CHANNELS_PER_SLAVE];
	int irevol[MAX_NUMBER_OF_ENC_CHANNELS_PER_SLAVE];
	int prev_count[MAX_NUMBER_OF_ENC_CHANNELS_PER_SLAVE];
} encoder_data_t;

#define EL_5101_COUNTER_MAX 65536
#define EL_5152_COUNTER_MAX 4294967296
