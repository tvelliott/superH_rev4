typedef struct {
	unsigned char mode;
	unsigned char addr[8];
	unsigned char ack;
	unsigned char label[3];
	unsigned char bid;
	unsigned char no[5];
	unsigned char fid[7];
	char txt[280];
  int tot_len;
} msg_t;

extern int getsample(short *sample, int nb);
extern void init_bits(void);
extern void resetbits(int ch);
extern int getbit(short in, unsigned char *outbits, int ch);
extern void init_mesg(void);
extern int getmesg(unsigned char r, msg_t * msg, int ch);
