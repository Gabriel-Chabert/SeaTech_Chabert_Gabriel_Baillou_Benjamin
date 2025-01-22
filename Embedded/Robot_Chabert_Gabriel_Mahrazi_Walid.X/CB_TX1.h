#ifndef CB_TX1_H
#define	CB_TX1_H
#define CBTX1_BUFFER_SIZE 128
void SendMessage(unsigned char*,int);
void CB_TX1_Add(unsigned char);
unsigned char CB_TX1_Get(void);
void SendOne();
unsigned char CB_TX1_IsTranmitting(void);
int CB_TX1_GetDataSize(void);
int CB_TX1_GetRemainingSize(void);


#endif	/* CB_TX1_H */

