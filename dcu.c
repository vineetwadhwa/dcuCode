#include"header.h"
#include <math.h>

//******************************************SSC Firmware Version Declaration*********************************************/

char *VersionNumber = " SSC Version -13 ";


/******************************************SSC Macros Declaration********************************************************/

#define METERINFO_FILE_PATH "/home/pi/pushpollDcu/pushpoll_BBB/meterinfo.txt"
#define Enable_Single_Thread_MSGQ
#define Enable_NodeList_Info
#define AppendMacFromArray
#define Enable_Arguments
#define ENABLE_COMPARE_MAC
#define Debug_Print_Enable_LAPA
#define ENABLE_PUSHRESPONSE_PI
#define SEND_NR
#define Enable_PUSH_And_Response_Print
#define Enable_BBB
#define MAX 500
#define SSC_CURL
#define PUSH_VIA_CURL_SQL

#define PUSH_PCKT_PRINTF_TEST
#define ipv6_poll

#define ESW_BIT_LEN    16

//#define Debug_Print_Enable
//#define Enable_Ping
//#define Enable_Dcu_Reset
//#define ENABLE_FD_LEAKAGE_DEBUG
//#define Debug_Print_Enable_WriteTh
//#define Two_Uart_Enable
//#define debug_curl_print



//******************************************SSC Function Prototypes Declaration******************************************/


void insertPollRequest();
void delete();
void display();
int perform_curl_operation(char* data, int len);
int perform_curl_operation_SinglePhasepush(char* data, int len);
int perform_curl_operation_ThreePhasepush(char* data, int len);
int perform_curl_operation_SinglePhaseAlert(char* data, int len);
int perform_curl_operation_ThreePhaseAlert(char* data, int len);
int perform_curl_operation_error(char* data, int len);


//******************************************SSC Global Structures Declaration******************************************/

//structure Declaration
struct mesg_buffer
{
    long mesg_type;
    char mesg_text[200];
} message;


struct request
{
    int socket_id;
    int frame_length;
    char frame_bytes[100];
};

typedef struct
{
    uint8 MeterNum[7];
    uint8 PushDataObis[25];
    uint8 rtc[20];
    uint16 LSActiveEnergy;
    uint16 LSApparentEnergy;
    uint16 LSAvgVoltage;
    uint16 LSAvgCurrent;
    uint16 InstantVoltage;
    uint16 InstantPhaseCurrent;
    uint16 InstantNeutralCurrent;
    int32 InstantActivePower;
    int32 InstantApparentPower;
    uint32 CumActiveEnergy;
    uint32 CumApparentEnergy;
    uint16 SignalStrength;
    uint16 RFParentAddr;
    uint16 RFHopValue;
    uint16 RFCoordinatorAddr;
    uint16 RFMacAddr;
    uint32 prepaidBal;
    uint16 frequency;
    uint16 totalEventCount;
    uint16 billingCount;
    uint8 relayStatus;
    uint8 loadLimitValue;
    uint32 meterOnTime;
    uint16 maxActiveDemand;
    uint16 maxApparentDemand;
    int32 InstantReactivePower;
    uint16 prepaidReferenceID;
    uint8 paramSize;
} singlepushdata;
singlepushdata SinglePushData;




typedef struct
{
    uint8 MeterNum[7];
    uint8 PushDataObis[25];
    uint8 rtc[20];
    uint16 LSActiveEnergy;
    uint16 LSReactLagEnergy;
    uint16 LSReactLeadEnergy;
    uint16 LSApparentEnergy;
    uint16 LSVoltageR;
    uint16 LSVoltageY;
    uint16 LSVoltageB;
    uint16 LSCurrentR;
    uint16 LSCurrentY;
    uint16 LSCurrentB;
    int32 InstantNeutralCurrent;
    uint32 CumActiveEnergy;
    uint32 CumReactLagEnergy;
    uint32 CumReactLeadEnergy;
    uint32 CumApparentEnergy;
    uint16 SignalStrength;
    uint16 RFCoordinatorAddr;
    uint16 RFParentAddr;
    uint16 RFMacAddr;
    uint16 frequency;
    uint32 prepaidBal;
    uint16 prepaidReferenceID;
    uint16 InstantVoltageR;
    uint16 InstantVoltageY;
    uint16 InstantVoltageB;
    uint16 InstantCurrentR;
    uint16 InstantCurrentY;
    uint16 InstantCurrentB;
    uint16 InstantPowerFactorR;
    uint16 InstantPowerFactorY;
    uint16 InstantPowerFactorB;
    uint16 InstantPowerFactorThree;
    uint8 paramSize;
} threepushdata;
threepushdata ThreePushData;

typedef struct
{
    uint8 MeterNum[7];
    uint8 PushDataObis[25];
    uint8 rtc[20];
    uint8 EswBit[ESW_BIT_LEN*8+1];
    uint16 RFCoordinatorAddr;
    uint8 hexStr[ESW_BIT_LEN*2 + 1];// not use in alert push parameter
    uint16 EventCode;
    uint16 InstantVoltage;
    uint16 InstantPhaseCurrent;
    uint16 PowerFactor;
    uint32 CumActiveEnergy;
    uint16 totalEventCount;
    uint8 paramSize;
} alertsinglepushdata;
alertsinglepushdata AlertSinglePushData;

typedef struct
{
    uint8 MeterNum[7];
    uint8 PushDataObis[25];
    uint8 rtc[20];
    uint8 EswBit[ESW_BIT_LEN*8+1];
    uint16 RFCoordinatorAddr;
    uint8 hexStr[ESW_BIT_LEN*2 + 1];// not use in alert push parameter
    uint16 EventCode;
    uint16 InstantCurrentR;
    uint16 InstantCurrentY;
    uint16 InstantCurrentB;
    uint16 InstantVoltageR;
    uint16 InstantVoltageY;
    uint16 InstantVoltageB;
    uint16 InstantPowerFactorR;
    uint16 InstantPowerFactorY;
    uint16 InstantPowerFactorB;
    uint32 CumActiveEnergy;
    uint16 totalEventCount;
    uint8 paramSize;
} alertthreepushdata;
alertthreepushdata AlertThreePushData;

/******************************************SSC Global Variables Declaration***********************************************/



unsigned int fcstab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,//7
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,//15
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,//23
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,//31
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,//39
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,//47
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,//55
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,//63
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,//71
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,//79
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,//87
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,//95
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,//103
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,//111
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,//119
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,//127
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,//135
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,//143
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,//151
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,//159
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,//167
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,//175
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,//183
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,//191
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,//199
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,//207
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,//215
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,//223
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,//231
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,//239
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,//247
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78//255
};


int nums[50];
int HESsfd[50];
int PIfd[50];
int HEScfd[50];
unsigned char meterTag[500];
int threadCount = 8,nodeAddr=0;
int fdUART1;
char *pushipAddr;
int pushPort;
int PushRespPort = 0;
ssize_t checkreturn ;
ssize_t checkreturn2 ;
key_t key;
int msgid;
char request_buffer[200]= {'\0'};
char read_buffer[200]= {'\0'};
unsigned char response_buffer[500]= {'\0'};
char fcsarr[2];
struct request queue_array[MAX];
int rear = - 1;
int front = - 1;
ssize_t return_writhTH;
pthread_t tid[100];
pthread_mutex_t lock;
int Push_HesDisconnect=0;


//******************************************SSC MACRO Based Global Variables Declaration*****************************************/




#ifdef Enable_Dcu_Reset
#define PUSH_INTERVAL (5*60) //in seconds
clock_t prev_push_time;
#endif



#ifdef Enable_BBB
//need port to connect 40000
int BBB_PORT = 40000;
int BBB_Socket_Disconnect = 1;
int BBB_fd;
enum {BBB_PUSH_RESPONSE = 1, BBB_POLL_RESPONSE, BBB_RESPONSE_UNKNOWN, BBB_NR_RESPONSE};
#define BBB_BUFFER_SIZE 234
uint8_t BBB_rbuff[BBB_BUFFER_SIZE];
#define BBB_TIMEOUT	5 //in seconds
#endif


#ifdef AppendMacFromArray
//for LAPA and MAC ARRAY
#define LAPA_MAC_ARRAY_SIZE	1000 //800
char ARRAY_LAPA_MAC[LAPA_MAC_ARRAY_SIZE][16];
char ARRAY_LAPA_MID[LAPA_MAC_ARRAY_SIZE][8];
#endif


#ifdef Enable_Arguments
int ssh_port = 0;
#endif


#define PUSHRESPONSE_INTERVAL_SIZE	0x92
#define PUSHRESPONSE_TAMPER_SIZE_OLD	0x65
#define PUSHRESPONSE_TAMPER_SIZE	0x68
#define PUSHRESPONSE_NEW_SINGLE_TAMPER_SIZE 0x7B
#define PUSHRESPONSE_NEW_THREE_TAMPER_SIZE 0x8B
uint8_t pushResponse_interval[14] =
{
    0x7E,0xA0,0x0C,0x00,0x00,0x00,0x00,0x24,0x50,0x52,0x78,0x31,0x23,0x7E
};
uint8_t pushResponse_interval_old[13] =
{
    0x7E,0xA0,0x0B,0x00,0x00,0x00,0x00,0x24,0x50,0x52,0x78,0x23,0x7E
};
uint8_t pushResponse_tamper[14] =
{
    0x7E,0xA0,0x0C,0x00,0x00,0x00,0x00,0x24,0x50,0x52,0x78,0x32,0x23,0x7E
};


#ifdef SEND_NR
#define NR_PUSH_TIME	60 //in seconds
int pushfd;
int NR = 0;
int nr_push(int num_route, char* meterTitle_params);
int push_count=0;
uint8_t Finaltxbuf[150];
extern unsigned char Enc_Data_Buffer[200];
#include "AES_GCM_128.h"
//int16_t errCode;
int dcu_id = 0;
uint8_t dcu_id_push[2] = {0};
unsigned char meterTitle[8] = {0};
#endif

#ifdef SSC_CURL
#include <curl/curl.h>
#define STAGING_URL  "http://3.6.200.99:5001/api/routeInfo"
#define STAGING_URL_PUSH_SinglePhase 	"http://3.6.200.99:5001/api/curlsinglephasedata"
#define STAGING_URL_PUSH_ThreePhase 	"http://3.6.200.99:5001/api/curlthreephasedata"
#define STAGING_URL_PUSH_SinglePhaseAlert 	"http://3.6.200.99:5001/api/curlsinglephasetamper"
#define STAGING_URL_PUSH_ThreePhaseAlert 	"http://3.6.200.99:5001/api/curlthreephasetamper"
#define CURL_ERROR_URL						"http://3.6.200.99:5001/api/curlerror"

//#define STAGING_URL_PUSH "http://65.2.81.178:5001/api/curlReceiptMizo"

#endif


#ifdef PUSH_VIA_CURL_SQL

#define ENCRYPTION_KEY        "LogicEasternIndL" // define this macro in AES_GCM_128.h
#define ENCRYPT   1
#define DECRYPT   0
/*
//#define JNJ
//#define MSE
//#define PCP
//#define ITI
//#define IBE
#define LEI

#ifdef JNJ
uint8 pushClientSystemTitle[]={ 0x4A, 0x4E, 0x4A, 0x30, 0x33, 0x36, 0x30, 0x37};
#endif

#ifdef MSE
uint8 pushClientSystemTitle[]={0x4D, 0x53, 0x45, 0x30, 0x33, 0x36, 0x30, 0x37};
#endif

#ifdef PCP
uint8 pushClientSystemTitle[]={ 0x4C, 0x45, 0x49, 0x30, 0x33, 0x36, 0x30, 0x37};
#endif

#ifdef ITI
uint8 pushClientSystemTitle[]={0x49, 0x54, 0x50, 0x30, 0x33, 0x36, 0x30, 0x37};
#endif

#ifdef IBE
uint8 pushClientSystemTitle[]={0x49, 0x42, 0x45, 0x30, 0x33, 0x36, 0x30, 0x37};
#endif

#ifdef LEI
uint8 pushClientSystemTitle[]={0x4C, 0x45, 0x49, 0x30, 0x33, 0x36, 0x30, 0x37};
#endif
*/

extern unsigned char Enc_Data_Buffer[200];
unsigned char PushDecryptedBuffer[200];
void DecryptPush(char PushPacket[], int PushLength);
void SetDefaultPush(void);
void HexStringToBitCharStringSingle(uint8_t* input, uint8_t* output);
void HexStringToBitCharStringThree(uint8_t* input, uint8_t* output);

// 21 parameter Push Single Phase
char PushEncryptedBuffer[] = {0x7e, 0xa0, 0x92, 0xfe, 0xfe, 0xfe, 0xfe, 0x00, 0x01, 0x00, 0x01\
                              , 0x00, 0x40, 0x00, 0x82, 0xdb, 0x08, 0x4d, 0x53, 0x45, 0x30, 0x30, 0x30, 0x33, 0x34, 0x77\
                              , 0x20, 0x00, 0x00, 0x00, 0x00, 0xdb, 0xed, 0x97, 0xfd, 0xcf, 0x9f, 0x89, 0x43, 0xf7, 0x17\
                              , 0x1e, 0x0b, 0xc2, 0x43, 0xc5, 0xbb, 0x43, 0xb3, 0x74, 0x93, 0x69, 0x7e, 0x6c, 0xa0, 0xcd\
                              , 0x0f, 0x03, 0x10, 0x5a, 0xd1, 0xff, 0x54, 0xa3, 0xfb, 0x37, 0x8c, 0x7b, 0xbb, 0xae, 0x17\
                              , 0xf5, 0x31, 0x12, 0xaa, 0xc3, 0xd4, 0x6a, 0x34, 0xfb, 0x0c, 0x24, 0xc7, 0xc0, 0x41, 0xe4\
                              , 0x5d, 0x54, 0x3c, 0x74, 0xaf, 0xd3, 0x2f, 0x5c, 0xd4, 0xf0, 0x23, 0xe3, 0xc2, 0x5a, 0x20\
                              , 0xe2, 0xeb, 0x1e, 0x67, 0x4b, 0xc2, 0x6f, 0x9e, 0x8c, 0x86, 0x70, 0xc0, 0xe4, 0xe6, 0x78\
                              , 0x9f, 0x61, 0x66, 0x07, 0x35, 0x5e, 0xd9, 0x17, 0xac, 0x85, 0xdb, 0x92, 0x33, 0xc9, 0x92\
                              , 0x79, 0x61, 0x2f, 0x9b, 0xb6, 0xd5, 0xe5, 0xb4, 0x25, 0x99, 0x9e, 0x7f, 0xfa, 0x17, 0x9e\
                              , 0xa6, 0x7e
                             };


/* // 18 parameter Push Single Phase
char PushEncryptedBuffer[] = {0x7E, 0xA0, 0x82, 0xFE, 0xFE, 0xFE, 0xFE, 0x00, 0x01\
, 0x00, 0x01, 0x00, 0x40, 0x00, 0x72, 0xDB, 0x08, 0x4C, 0x45, 0x49, 0x30, 0x33, 0x36\
, 0x30, 0x37, 0x67, 0x20, 0x00, 0x00, 0x00, 0x00, 0x13, 0x0B, 0x14, 0x32, 0x90, 0x55\
, 0xE1, 0x76, 0x45, 0xA3, 0xD7, 0x60, 0x28, 0x15, 0x68, 0x6B, 0xB4, 0xAC, 0xFB, 0x0C\
, 0x0E, 0xA5, 0xAB, 0xF4, 0x11, 0xD9, 0x23, 0xAE, 0x0B, 0xDD, 0x3A, 0x48, 0x9E, 0x83\
, 0x0B, 0x70, 0x91, 0x60, 0x18, 0x88, 0x15, 0x6F, 0x41, 0x0D, 0x26, 0x11, 0x4D, 0x04\
, 0x33, 0x1B, 0x63, 0x6B, 0x94, 0xF7, 0x96, 0xD3, 0x3F, 0x82, 0xFA, 0x7F, 0x2B, 0x7E\
, 0xAA, 0xE8, 0xC6, 0xE9, 0xE7, 0x76, 0xFB, 0x9D, 0xBA, 0xD0, 0x63, 0xA9, 0x93, 0xEC\
, 0xA6, 0x9C, 0xFE, 0x3D, 0x93, 0x5A, 0x35, 0x19, 0x2A, 0x56, 0x67, 0xF6, 0x4F, 0x9A\
, 0x19, 0xCD, 0x92, 0x54, 0xFE, 0x34, 0x2A, 0xD1, 0xC4, 0x68, 0x7E};
*/
#endif



#ifdef PUSH_VIA_CURL_SQL

void DecryptPush(char PushPacket[], int PushLength)
{
    unsigned char iv[12] = {0x00};
    unsigned char DecryptLength;
    uint8 i=0;
    unsigned char tempStr[25] = {0x00};
    unsigned char tempConvert[5] = {0x00};
    uint16 tempYear;
    uint8 tempHH,tempMM,tempSS,tempMon,tempDay;
    char data[1000] = {'\0'};

    printf("\n  ******************************PushLength = %d ***************************\n",PushLength);

    SetDefaultPush();

    DecryptLength = PushLength - 34; // Encrypted Packet Length
    //memcpy(iv,pushClientSystemTitle,8);
    memcpy(iv,&PushPacket[17],8);
    errCode=1;
    enc_dec(&PushPacket[31],DecryptLength,iv,NULL,0,DECRYPT);
    errCode=0;
    memcpy(PushDecryptedBuffer,&Enc_Data_Buffer[18],sizeof(PushDecryptedBuffer));

#ifdef PUSH_PCKT_PRINTF_TEST
    printf("\n Decrypted Push Packet Data\n");
    for(uint8 j=0; j < (PushLength - 18); j++)
        printf("%x ", PushDecryptedBuffer[j]);
#endif
    SinglePushData.RFCoordinatorAddr = dcu_id; // dcu id fill by this code
    ThreePushData.RFCoordinatorAddr = dcu_id; // dcu id fill by this code
    AlertSinglePushData.RFCoordinatorAddr = dcu_id; // dcu id fill by this code
    AlertThreePushData.RFCoordinatorAddr = dcu_id; // dcu id fill by this code

    //if((PushDecryptedBuffer[13] == 0x00) && (PushDecryptedBuffer[16] == 0x00))
    if(PushDecryptedBuffer[16] == 0x00) // For Single Phase, where PUSH OBIS = 0x00 0x00 0x19 0x09 0x00 0xFF
    {
        if((PushLength == 103) || (PushLength == 106) || (PushLength == 125)) // Push By Alert
        {
            printf(" \n ***********************SINGLE PHASE TAMPER DATA********************* \n ");
            i = 4; //4
            memcpy(AlertSinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

            i += 8; //12
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
            strcpy(AlertSinglePushData.PushDataObis,tempConvert);
            strcat(AlertSinglePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
            strcat(AlertSinglePushData.PushDataObis,tempConvert);
            strcat(AlertSinglePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
            strcat(AlertSinglePushData.PushDataObis,tempConvert);
            strcat(AlertSinglePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
            strcat(AlertSinglePushData.PushDataObis,tempConvert);
            strcat(AlertSinglePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
            strcat(AlertSinglePushData.PushDataObis,tempConvert);
            strcat(AlertSinglePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
            strcat(AlertSinglePushData.PushDataObis,tempConvert);

            i += 8; //20
            tempHH = PushDecryptedBuffer[i]; // Hour
            i += 1; //21
            tempMM = PushDecryptedBuffer[i]; // Min
            i += 1; //22
            tempSS = PushDecryptedBuffer[i]; // Sec
            i += 2; //24
            tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
            i += 2; //26
            tempMon = PushDecryptedBuffer[i]; // month
            i += 1; //27
            tempDay = PushDecryptedBuffer[i]; // Day
            sprintf(tempConvert, "%d", tempYear);
            strcpy(AlertSinglePushData.rtc,tempConvert);
            strcat(AlertSinglePushData.rtc,"-");
            sprintf(tempConvert, "%02ld", tempMon);
            strcat(AlertSinglePushData.rtc,tempConvert);
            strcat(AlertSinglePushData.rtc,"-");
            sprintf(tempConvert, "%02ld", tempDay);
            strcat(AlertSinglePushData.rtc,tempConvert);
            strcat(AlertSinglePushData.rtc," ");
            sprintf(tempConvert, "%02ld", tempHH);
            strcat(AlertSinglePushData.rtc,tempConvert);
            strcat(AlertSinglePushData.rtc,":");
            sprintf(tempConvert, "%02ld", tempMM);
            strcat(AlertSinglePushData.rtc,tempConvert);
            strcat(AlertSinglePushData.rtc,":");
            sprintf(tempConvert, "%02ld", tempSS);
            strcat(AlertSinglePushData.rtc,tempConvert);

            i += 8; //35
            HexStringToBitCharStringSingle(&PushDecryptedBuffer[i],AlertSinglePushData.hexStr);
            printf("\nAlert Hex Bytes = %s\n", AlertSinglePushData.hexStr);

            if(PushLength == 125) // New Tamper Push
            {
                i += 17; //52
                AlertSinglePushData.EventCode = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //55
                AlertSinglePushData.InstantVoltage = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //58
                AlertSinglePushData.InstantPhaseCurrent = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //61
                AlertSinglePushData.PowerFactor = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //64
                AlertSinglePushData.CumActiveEnergy = ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //69
                AlertSinglePushData.totalEventCount = ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                AlertSinglePushData.paramSize = 10;
            }
            else
                AlertSinglePushData.paramSize = 4;

            sprintf(data,
                    "&meterNumber=%s"
                    "&pushObis=%s"
                    "&rtc=%s"
                    "&esw=%s"
                    "&dcuId=%u"
                    "&eventCode=%u"
                    "&instantVoltage=%u"
                    "&instantPhaseCurrent=%u"
                    "&instantPowerFactor=%u"
                    "&activeEnergy=%u"
                    "&totalEventCount=%u"
                    "&paramSize=%u"
                    ,
                    AlertSinglePushData.MeterNum,AlertSinglePushData.PushDataObis,AlertSinglePushData.rtc,AlertSinglePushData.EswBit
                    ,AlertSinglePushData.RFCoordinatorAddr,AlertSinglePushData.EventCode,AlertSinglePushData.InstantVoltage,AlertSinglePushData.InstantPhaseCurrent
                    ,AlertSinglePushData.PowerFactor,AlertSinglePushData.CumActiveEnergy,AlertSinglePushData.totalEventCount,AlertSinglePushData.paramSize);
            perform_curl_operation_SinglePhaseAlert(data,strlen(data));
        }
        else // Push by Interval
        {
            if(PushLength == 148) // 21 RF Parameter
            {
                SinglePushData.paramSize = 21;
                i = 4; //4
                memcpy(SinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(SinglePushData.PushDataObis,tempConvert);


                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(SinglePushData.rtc,tempConvert);

                i += 6; //33
                SinglePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                SinglePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                SinglePushData.LSAvgVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                SinglePushData.LSAvgCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                SinglePushData.InstantVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                SinglePushData.InstantPhaseCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                SinglePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //54
                SinglePushData.InstantActivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //59
                SinglePushData.InstantReactivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //64
                SinglePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //69
                SinglePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //74
                SinglePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //77
                SinglePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //80
                SinglePushData.RFParentAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //83
                SinglePushData.RFHopValue =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //86
                // SinglePushData.RFCoordinatorAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //89
                SinglePushData.prepaidReferenceID =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //92
                SinglePushData.prepaidBal =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);
            }
            else if(PushLength == 143) // 20 RF Parameter
            {
                SinglePushData.paramSize = 20;
                i = 4; //4
                memcpy(SinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(SinglePushData.PushDataObis,tempConvert);


                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(SinglePushData.rtc,tempConvert);

                i += 6; //33
                SinglePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                SinglePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                SinglePushData.LSAvgVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                SinglePushData.LSAvgCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                SinglePushData.InstantVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                SinglePushData.InstantPhaseCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                SinglePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //54
                SinglePushData.InstantActivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //59
                SinglePushData.InstantReactivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //64
                SinglePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //69
                SinglePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //74
                SinglePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //77
                SinglePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //80
                SinglePushData.RFParentAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //83
                SinglePushData.RFHopValue =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //86
                //SinglePushData.RFCoordinatorAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //89
                SinglePushData.RFMacAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);
            }
            else if(PushLength == 132) // 18 RF Parameter
            {
                SinglePushData.paramSize = 18;
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                i = 4; //4
                memcpy(SinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcpy(SinglePushData.PushDataObis,tempConvert);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,".");
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,tempConvert);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,".");
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,tempConvert);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,".");
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,tempConvert);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,".");
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,tempConvert);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,".");
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
                strcat(SinglePushData.PushDataObis,tempConvert);
                //printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);

                i += 7; //19
                SinglePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //22
                SinglePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //25
                SinglePushData.LSAvgVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //28
                SinglePushData.LSAvgCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //31
                SinglePushData.InstantVoltage =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //36
                SinglePushData.InstantPhaseCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //41
                SinglePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //46
                SinglePushData.InstantActivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //51
                SinglePushData.InstantReactivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //56
                SinglePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //61
                SinglePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //66
                SinglePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //69
                SinglePushData.RFParentAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //72
                SinglePushData.RFHopValue =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //75
                // SinglePushData.RFCoordinatorAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //78
                SinglePushData.RFMacAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                printf("\n func = %s and line = %d \n", __FUNCTION__,__LINE__);
            }
            else if(PushLength == 126) // 17 GSM Parameter
            {
                SinglePushData.paramSize = 17;
                i = 4; //4
                memcpy(SinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(SinglePushData.PushDataObis,tempConvert);

                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(SinglePushData.rtc,tempConvert);

                i += 6; //33
                SinglePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                SinglePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                SinglePushData.LSAvgVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                SinglePushData.LSAvgCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                SinglePushData.InstantVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                SinglePushData.InstantPhaseCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                SinglePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //54
                SinglePushData.InstantActivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //59
                SinglePushData.InstantReactivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //64
                SinglePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //69
                SinglePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //74
                SinglePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //77
                SinglePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //80
                SinglePushData.prepaidBal =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);
            }
            else if(PushLength == 130) // 13 RF Parameter
            {
                SinglePushData.paramSize = 13;
                i = 7; //7
                memcpy(SinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //15
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(SinglePushData.PushDataObis,tempConvert);


                i += 8; //23
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //24
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //25
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //27
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //29
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //30
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(SinglePushData.rtc,tempConvert);

                i += 6; //36
                SinglePushData.InstantVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                SinglePushData.InstantPhaseCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                SinglePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //47
                SinglePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //52
                SinglePushData.maxActiveDemand =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //57
                SinglePushData.maxApparentDemand =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //62
                SinglePushData.totalEventCount =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //67
                SinglePushData.billingCount =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //72
                SinglePushData.relayStatus =  PushDecryptedBuffer[i];

                i += 2; //74
                SinglePushData.loadLimitValue =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);
            }
            else if(PushLength == 105) // 11 GSM Parameter
            {
                SinglePushData.paramSize = 11;
                i = 4; //4
                memcpy(SinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(SinglePushData.PushDataObis,tempConvert);

                i += 8; //20
                SinglePushData.InstantVoltage =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //25
                SinglePushData.InstantPhaseCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //30
                SinglePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //35
                SinglePushData.InstantActivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //40
                SinglePushData.InstantReactivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //45
                SinglePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //50
                SinglePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //55
                SinglePushData.meterOnTime =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //60
                SinglePushData.totalEventCount =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);
            }
            else
            {
                i = 4; //4
                memcpy(SinglePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(SinglePushData.PushDataObis,tempConvert);
                strcat(SinglePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(SinglePushData.PushDataObis,tempConvert);


                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(SinglePushData.rtc,tempConvert);
                strcat(SinglePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(SinglePushData.rtc,tempConvert);

                i += 6; //33
                SinglePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                SinglePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                SinglePushData.LSAvgVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                SinglePushData.LSAvgCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                SinglePushData.InstantVoltage =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                SinglePushData.InstantPhaseCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                SinglePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //54
                SinglePushData.InstantActivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //59
                SinglePushData.InstantReactivePower =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //64
                SinglePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //69
                SinglePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //74
                SinglePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //77
                SinglePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //80
                SinglePushData.RFParentAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //83
                SinglePushData.RFHopValue =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //86
                // SinglePushData.RFCoordinatorAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //89
                SinglePushData.prepaidReferenceID =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //92
                SinglePushData.prepaidBal =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);
            }

#ifdef debug_curl_print
            printf("\nData = %s\n", SinglePushData.MeterNum);
            printf("\nData = %s\n", SinglePushData.PushDataObis);
            printf("\nData = %s\n", SinglePushData.rtc);
#endif

            sprintf(tempStr,"%u", SinglePushData.LSActiveEnergy);
            sprintf(tempStr,"%u", SinglePushData.LSApparentEnergy);
            sprintf(tempStr,"%u", SinglePushData.LSAvgVoltage);
            sprintf(tempStr,"%u", SinglePushData.LSAvgCurrent);
            sprintf(tempStr,"%u", SinglePushData.InstantVoltage);
            sprintf(tempStr,"%u", SinglePushData.InstantPhaseCurrent);
            sprintf(tempStr,"%u", SinglePushData.InstantNeutralCurrent);
            sprintf(tempStr,"%u", SinglePushData.InstantActivePower);
            sprintf(tempStr,"%u", SinglePushData.InstantApparentPower);
            sprintf(tempStr,"%u", SinglePushData.CumActiveEnergy);
            sprintf(tempStr,"%u", SinglePushData.CumApparentEnergy);
            sprintf(tempStr,"%u", SinglePushData.frequency);
            sprintf(tempStr,"%u", SinglePushData.SignalStrength);
            sprintf(tempStr,"%u", SinglePushData.RFParentAddr);
            sprintf(tempStr,"%u", SinglePushData.RFHopValue);
            sprintf(tempStr,"%u", SinglePushData.RFMacAddr);
            sprintf(tempStr,"%u", SinglePushData.RFCoordinatorAddr);
            sprintf(tempStr,"%u", SinglePushData.prepaidBal);
            sprintf(tempStr,"%u", SinglePushData.totalEventCount);
            sprintf(tempStr,"%u", SinglePushData.billingCount);
            sprintf(tempStr,"%u", SinglePushData.relayStatus);
            sprintf(tempStr,"%u", SinglePushData.loadLimitValue);
            sprintf(tempStr,"%u", SinglePushData.meterOnTime);
            sprintf(tempStr,"%u", SinglePushData.maxActiveDemand);
            sprintf(tempStr,"%u", SinglePushData.maxApparentDemand);
            sprintf(tempStr,"%u", SinglePushData.InstantReactivePower);
            sprintf(tempStr,"%u", SinglePushData.prepaidReferenceID);
            sprintf(tempStr,"%u", SinglePushData.paramSize);

            sprintf(data,
                    "&meterNumber=%s"
                    "&pushObis=%s"
                    "&rtc=%s"
                    "&LSActiveEnergy=%u"
                    "&LSApparentEnergy=%u"
                    "&LSVoltage=%u"
                    "&LSCurrent=%u"
                    "&instantVoltage=%u"
                    "&instantPhaseCurrent=%u"
                    "&instantNeutralCurrent=%u"
                    "&activePower=%d"
                    "&apparentPower=%d"
                    "&activeEnergy=%u"
                    "&apparentEnergy=%u"
                    "&frequency=%u"
                    "&signalStrength=%u"
                    "&parentAddress=%u"
                    "&hopValue=%u"
                    "&coordinatorID=%u"
                    "&selfMacAddress=%u"
                    "&prepaidBalance=%u"
                    "&totalEventCount=%u"
                    "&billingCount=%u"
                    "&relayStatus=%u"
                    "&loadLimitValue=%u"
                    "&meterOnTime=%u"
                    "&maxActiveDemand=%u"
                    "&maxApparentDemand=%u"
                    "&reactivePower=%d"
                    "&prepaidReferenceID=%d"
                    "&paramSize=%d"
                    ,
                    SinglePushData.MeterNum,SinglePushData.PushDataObis,SinglePushData.rtc,SinglePushData.LSActiveEnergy,SinglePushData.LSApparentEnergy
                    ,SinglePushData.LSAvgVoltage,SinglePushData.LSAvgCurrent,SinglePushData.InstantVoltage,SinglePushData.InstantPhaseCurrent
                    ,SinglePushData.InstantNeutralCurrent,SinglePushData.InstantActivePower,SinglePushData.InstantApparentPower
                    ,SinglePushData.CumActiveEnergy,SinglePushData.CumApparentEnergy,SinglePushData.frequency,SinglePushData.SignalStrength
                    ,SinglePushData.RFParentAddr,SinglePushData.RFHopValue,SinglePushData.RFCoordinatorAddr,SinglePushData.RFMacAddr,SinglePushData.prepaidBal
                    ,SinglePushData.totalEventCount,SinglePushData.billingCount,SinglePushData.relayStatus,SinglePushData.loadLimitValue,SinglePushData.meterOnTime
                    ,SinglePushData.maxActiveDemand,SinglePushData.maxApparentDemand,SinglePushData.InstantReactivePower,SinglePushData.prepaidReferenceID
                    ,SinglePushData.paramSize);

            perform_curl_operation_SinglePhasepush(data,strlen(data));
        }
    }
    else // For Three Phase
    {
        if((PushLength == 103) || (PushLength == 106) || (PushLength == 141)) // Push By Alert
        {
            printf(" \n ***********************THREE PHASE TAMPER DATA********************* \n ");
            i = 4; //4
            memcpy(AlertThreePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

            i += 8; //12
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
            strcpy(AlertThreePushData.PushDataObis,tempConvert);
            strcat(AlertThreePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
            strcat(AlertThreePushData.PushDataObis,tempConvert);
            strcat(AlertThreePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
            strcat(AlertThreePushData.PushDataObis,tempConvert);
            strcat(AlertThreePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
            strcat(AlertThreePushData.PushDataObis,tempConvert);
            strcat(AlertThreePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
            strcat(AlertThreePushData.PushDataObis,tempConvert);
            strcat(AlertThreePushData.PushDataObis,".");
            sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
            strcat(AlertThreePushData.PushDataObis,tempConvert);


            i += 8; //20
            tempHH = PushDecryptedBuffer[i]; // Hour
            i += 1; //21
            tempMM = PushDecryptedBuffer[i]; // Min
            i += 1; //22
            tempSS = PushDecryptedBuffer[i]; // Sec
            i += 2; //24
            tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
            i += 2; //26
            tempMon = PushDecryptedBuffer[i]; // month
            i += 1; //27
            tempDay = PushDecryptedBuffer[i]; // Day
            sprintf(tempConvert, "%d", tempYear);
            strcpy(AlertThreePushData.rtc,tempConvert);
            strcat(AlertThreePushData.rtc,"-");
            sprintf(tempConvert, "%02ld", tempMon);
            strcat(AlertThreePushData.rtc,tempConvert);
            strcat(AlertThreePushData.rtc,"-");
            sprintf(tempConvert, "%02ld", tempDay);
            strcat(AlertThreePushData.rtc,tempConvert);
            strcat(AlertThreePushData.rtc," ");
            sprintf(tempConvert, "%02ld", tempHH);
            strcat(AlertThreePushData.rtc,tempConvert);
            strcat(AlertThreePushData.rtc,":");
            sprintf(tempConvert, "%02ld", tempMM);
            strcat(AlertThreePushData.rtc,tempConvert);
            strcat(AlertThreePushData.rtc,":");
            sprintf(tempConvert, "%02ld", tempSS);
            strcat(AlertThreePushData.rtc,tempConvert);

            i += 8; //35
            HexStringToBitCharStringThree(&PushDecryptedBuffer[i],AlertThreePushData.hexStr);
            printf("\nAlert Hex Bytes = %s\n", AlertThreePushData.hexStr);

            if(PushLength == 141)
            {
                i += 17; // 52
                AlertThreePushData.EventCode =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 55
                AlertThreePushData.InstantCurrentR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 58
                AlertThreePushData.InstantCurrentY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 61
                AlertThreePushData.InstantCurrentB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 64
                AlertThreePushData.InstantVoltageR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 67
                AlertThreePushData.InstantVoltageY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 70
                AlertThreePushData.InstantVoltageB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 73
                AlertThreePushData.InstantPowerFactorR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 76
                AlertThreePushData.InstantPowerFactorY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 79
                AlertThreePushData.InstantPowerFactorB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; // 82
                AlertThreePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; // 87
                AlertThreePushData.totalEventCount =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                AlertThreePushData.paramSize = 16;
            }
            else
              AlertThreePushData.paramSize = 4;

            sprintf(data,
                    "&meterNumber=%s"
                    "&pushObis=%s"
                    "&rtc=%s"
                    "&esw=%s"
                    "&dcuId=%u"
                    "&eventCode=%u"
                    "&instCurrentR=%u"
                    "&instCurrentY=%u"
                    "&instCurrentB=%d"
                    "&instVoltageR=%u"
                    "&instVoltageY=%u"
                    "&instVoltageB=%u"
                    "&instPowerFactorR=%d"
                    "&instPowerFactorY=%d"
                    "&instPowerFactorB=%d"
                    "&activeEnergy=%u"
                    "&totalEventCount=%u"
                    "&paramSize=%u"
                    ,
                    AlertThreePushData.MeterNum,AlertThreePushData.PushDataObis,AlertThreePushData.rtc,AlertThreePushData.EswBit
                    ,AlertThreePushData.RFCoordinatorAddr,AlertThreePushData.EventCode,AlertThreePushData.InstantCurrentR
                    ,AlertThreePushData.InstantCurrentY,AlertThreePushData.InstantCurrentB,AlertThreePushData.InstantVoltageR
                    ,AlertThreePushData.InstantVoltageY,AlertThreePushData.InstantVoltageB,AlertThreePushData.InstantPowerFactorR
                    ,AlertThreePushData.InstantPowerFactorY,AlertThreePushData.InstantPowerFactorB,AlertThreePushData.CumActiveEnergy
                    ,AlertThreePushData.totalEventCount,AlertThreePushData.paramSize);
            perform_curl_operation_ThreePhaseAlert(data,strlen(data));
        }
        else // Push By Interval
        {
            if(PushLength == 156) // 23 RF Parameter
            {
                ThreePushData.paramSize = 23;
                i = 4; //4
                memcpy(ThreePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(ThreePushData.PushDataObis,tempConvert);


                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(ThreePushData.rtc,tempConvert);

                i += 6; //33
                ThreePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                ThreePushData.LSReactLagEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                ThreePushData.LSReactLeadEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                ThreePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                ThreePushData.LSVoltageR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                ThreePushData.LSVoltageY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                ThreePushData.LSVoltageB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //52
                ThreePushData.LSCurrentR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //55
                ThreePushData.LSCurrentY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //58
                ThreePushData.LSCurrentB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //61
                ThreePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //66
                ThreePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //71
                ThreePushData.CumReactLagEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //76
                ThreePushData.CumReactLeadEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //81
                ThreePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //86
                ThreePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //89
                //ThreePushData.RFCoordinatorAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //92
                ThreePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //95
                ThreePushData.prepaidBal =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //100
                ThreePushData.prepaidReferenceID =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);
            }
            else if(PushLength == 142) // 19 CCMS-PI RF Parameter
            {
                ThreePushData.paramSize = 19;
                i = 4; //4
                memcpy(ThreePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(ThreePushData.PushDataObis,tempConvert);


                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(ThreePushData.rtc,tempConvert);

                i += 6; //33
                ThreePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                ThreePushData.LSReactLagEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                ThreePushData.LSReactLeadEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                ThreePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                ThreePushData.LSVoltageR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                ThreePushData.LSVoltageY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                ThreePushData.LSVoltageB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //52
                ThreePushData.LSCurrentR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //55
                ThreePushData.LSCurrentY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //58
                ThreePushData.LSCurrentB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //61
                ThreePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //66
                ThreePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //71
                ThreePushData.CumReactLagEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //76
                ThreePushData.CumReactLeadEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //81
                ThreePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //86
                ThreePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);
            }
            else if(PushLength == 145) // 20 GSM and LTCT GSM Parameter
            {
                ThreePushData.paramSize = 20;
                i = 4; //4
                memcpy(ThreePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(ThreePushData.PushDataObis,tempConvert);


                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(ThreePushData.rtc,tempConvert);

                i += 6; //33
                ThreePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                ThreePushData.LSReactLagEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                ThreePushData.LSReactLeadEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                ThreePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                ThreePushData.LSVoltageR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                ThreePushData.LSVoltageY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                ThreePushData.LSVoltageB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //52
                ThreePushData.LSCurrentR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //55
                ThreePushData.LSCurrentY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //58
                ThreePushData.LSCurrentB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //61
                ThreePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //66
                ThreePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //71
                ThreePushData.CumReactLagEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //76
                ThreePushData.CumReactLeadEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //81
                ThreePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //86
                ThreePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //89
                ThreePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);
            }
            else
            {
                i = 4; //4
                memcpy(ThreePushData.MeterNum,&PushDecryptedBuffer[i],6); // Meter number

                i += 8; //12
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i]);
                strcpy(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+1]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+2]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+3]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+4]);
                strcat(ThreePushData.PushDataObis,tempConvert);
                strcat(ThreePushData.PushDataObis,".");
                sprintf(tempConvert, "%d", PushDecryptedBuffer[i+5]);
                strcat(ThreePushData.PushDataObis,tempConvert);


                i += 8; //20
                tempHH = PushDecryptedBuffer[i]; // Hour
                i += 1; //21
                tempMM = PushDecryptedBuffer[i]; // Min
                i += 1; //22
                tempSS = PushDecryptedBuffer[i]; // Sec
                i += 2; //24
                tempYear = ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]); // year
                i += 2; //26
                tempMon = PushDecryptedBuffer[i]; // month
                i += 1; //27
                tempDay = PushDecryptedBuffer[i]; // Day
                sprintf(tempConvert, "%d", tempYear);
                strcpy(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempMon);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,"-");
                sprintf(tempConvert, "%02ld", tempDay);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc," ");
                sprintf(tempConvert, "%02ld", tempHH);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempMM);
                strcat(ThreePushData.rtc,tempConvert);
                strcat(ThreePushData.rtc,":");
                sprintf(tempConvert, "%02ld", tempSS);
                strcat(ThreePushData.rtc,tempConvert);

                i += 6; //33
                ThreePushData.LSActiveEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //36
                ThreePushData.LSReactLagEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //39
                ThreePushData.LSReactLeadEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //42
                ThreePushData.LSApparentEnergy =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //45
                ThreePushData.LSVoltageR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //48
                ThreePushData.LSVoltageY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //51
                ThreePushData.LSVoltageB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //52
                ThreePushData.LSCurrentR =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //55
                ThreePushData.LSCurrentY =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //58
                ThreePushData.LSCurrentB =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //61
                ThreePushData.InstantNeutralCurrent =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //66
                ThreePushData.CumActiveEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //71
                ThreePushData.CumReactLagEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //76
                ThreePushData.CumReactLeadEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //81
                ThreePushData.CumApparentEnergy =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //86
                ThreePushData.SignalStrength =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //89
                //ThreePushData.RFCoordinatorAddr =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //92
                ThreePushData.frequency =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);

                i += 3; //95
                ThreePushData.prepaidBal =  ((PushDecryptedBuffer[i] << 24) | (PushDecryptedBuffer[i+1] << 16) | (PushDecryptedBuffer[i+2] << 8) | PushDecryptedBuffer[i+3]);

                i += 5; //100
                ThreePushData.prepaidReferenceID =  ((PushDecryptedBuffer[i] << 8) | PushDecryptedBuffer[i+1]);
            }

            sprintf(data,
                    "&meterNumber=%s"
                    "&pushObis=%s"
                    "&rtc=%s"
                    "&LSActiveEnergy=%u"
                    "&LSReactLagEnergy=%u"
                    "&LSReactLeadEnergy=%u"
                    "&LSApparentEnergy=%u"
                    "&LSVoltageR=%u"
                    "&LSVoltageY=%u"
                    "&LSVoltageB=%u"
                    "&LSCurrentR=%d"
                    "&LSCurrentY=%d"
                    "&LSCurrentB=%u"
                    "&instNeutralCurrent=%u"
                    "&activeEnergy=%u"
                    "&reactLagEnergy=%u"
                    "&reactLeadEnergy=%u"
                    "&apparentEnergy=%u"
                    "&signalStrength=%u"
                    "&coordinatorID=%u"
                    "&frequency=%u"
                    "&prepaidBalance=%u"
                    "&prepaidReferenceID=%u"
                    "&instVoltageR=%u"
                    "&instVoltageY=%u"
                    "&instVoltageB=%u"
                    "&instCurrentR=%u"
                    "&instCurrentY=%u"
                    "&instCurrentB=%d"
                    "&instPowerFactorR=%d"
                    "&instPowerFactorY=%d"
                    "&instPowerFactorB=%d"
                    "&instPowerFactorThreePhase=%d"
                    "&parentAddress=%d"
                    "&selfMacAddress=%d"
                    "&paramSize=%d"
                    ,
                    ThreePushData.MeterNum,ThreePushData.PushDataObis,ThreePushData.rtc,ThreePushData.LSActiveEnergy,ThreePushData.LSReactLagEnergy
                    ,ThreePushData.LSReactLeadEnergy,ThreePushData.LSApparentEnergy,ThreePushData.LSVoltageR,ThreePushData.LSVoltageY
                    ,ThreePushData.LSVoltageB,ThreePushData.LSCurrentR,ThreePushData.LSCurrentY,ThreePushData.LSCurrentB,ThreePushData.InstantNeutralCurrent
                    ,ThreePushData.CumActiveEnergy,ThreePushData.CumReactLagEnergy,ThreePushData.CumReactLeadEnergy,ThreePushData.CumApparentEnergy
                    ,ThreePushData.SignalStrength,ThreePushData.RFCoordinatorAddr,ThreePushData.frequency,ThreePushData.prepaidBal,ThreePushData.prepaidReferenceID
                    ,ThreePushData.InstantVoltageR,ThreePushData.InstantVoltageY,ThreePushData.InstantVoltageB,ThreePushData.InstantCurrentR,ThreePushData.InstantCurrentY
                    ,ThreePushData.InstantCurrentB,ThreePushData.InstantPowerFactorR,ThreePushData.InstantPowerFactorY,ThreePushData.InstantPowerFactorB
                    ,ThreePushData.InstantPowerFactorThree,ThreePushData.RFParentAddr,ThreePushData.RFMacAddr,ThreePushData.paramSize);
            perform_curl_operation_ThreePhasepush(data,strlen(data));
        }
    }
}
void SetDefaultPush(void)
{
    uint16 tempYear;
    uint8 tempHH,tempMM,tempSS,tempMon,tempDay;
    unsigned char tempConvert[5] = {0x00};

    memset(PushDecryptedBuffer,'\0',200);
    // Single Phase Meter
    memset(SinglePushData.MeterNum,0x00,6); // Meter number
    memcpy(SinglePushData.PushDataObis,"0.0.0.0.0.0",11); // Meter number

    tempHH = 0; // Hour
    tempMM = 0; // Min
    tempSS = 0; // Sec
    tempYear = 2022; // year
    tempMon = 1; // month
    tempDay = 1; // Day
    sprintf(tempConvert, "%d", tempYear);
    strcpy(SinglePushData.rtc,tempConvert);
    strcat(SinglePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempMon);
    strcat(SinglePushData.rtc,tempConvert);
    strcat(SinglePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempDay);
    strcat(SinglePushData.rtc,tempConvert);
    strcat(SinglePushData.rtc," ");
    sprintf(tempConvert, "%02ld", tempHH);
    strcat(SinglePushData.rtc,tempConvert);
    strcat(SinglePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempMM);
    strcat(SinglePushData.rtc,tempConvert);
    strcat(SinglePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempSS);
    strcat(SinglePushData.rtc,tempConvert);

    SinglePushData.billingCount = 0;
    SinglePushData.CumActiveEnergy = 0;
    SinglePushData.CumApparentEnergy = 0;
    SinglePushData.frequency = 0;
    SinglePushData.InstantActivePower = 0;
    SinglePushData.InstantApparentPower = 0;
    SinglePushData.InstantNeutralCurrent = 0;
    SinglePushData.InstantPhaseCurrent = 0;
    SinglePushData.InstantReactivePower = 0;
    SinglePushData.InstantVoltage = 0;
    SinglePushData.loadLimitValue = 0;
    SinglePushData.LSActiveEnergy = 0;
    SinglePushData.LSApparentEnergy = 0;
    SinglePushData.LSAvgCurrent = 0;
    SinglePushData.LSAvgVoltage = 0;
    SinglePushData.maxActiveDemand = 0;
    SinglePushData.maxApparentDemand = 0;
    SinglePushData.meterOnTime = 0;
    SinglePushData.prepaidBal = 0;
    SinglePushData.prepaidReferenceID = 0;
    SinglePushData.relayStatus = 0;
    SinglePushData.RFCoordinatorAddr = 0;
    SinglePushData.RFHopValue = 0;
    SinglePushData.RFMacAddr = 0;
    SinglePushData.RFParentAddr = 0;
    SinglePushData.SignalStrength = 0;
    SinglePushData.totalEventCount = 0;
    SinglePushData.paramSize =0;



    // Three Phase Meter
    memset(ThreePushData.MeterNum,0x00,6); // Meter number
    memcpy(ThreePushData.PushDataObis,"0.0.0.0.0.0",11); // Meter number

    tempHH = 0; // Hour
    tempMM = 0; // Min
    tempSS = 0; // Sec
    tempYear = 2022; // year
    tempMon = 1; // month
    tempDay = 1; // Day
    sprintf(tempConvert, "%d", tempYear);
    strcpy(ThreePushData.rtc,tempConvert);
    strcat(ThreePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempMon);
    strcat(ThreePushData.rtc,tempConvert);
    strcat(ThreePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempDay);
    strcat(ThreePushData.rtc,tempConvert);
    strcat(ThreePushData.rtc," ");
    sprintf(tempConvert, "%02ld", tempHH);
    strcat(ThreePushData.rtc,tempConvert);
    strcat(ThreePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempMM);
    strcat(ThreePushData.rtc,tempConvert);
    strcat(ThreePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempSS);
    strcat(ThreePushData.rtc,tempConvert);

    ThreePushData.CumActiveEnergy = 0;
    ThreePushData.CumApparentEnergy = 0;
    ThreePushData.CumReactLagEnergy = 0;
    ThreePushData.CumReactLeadEnergy = 0;
    ThreePushData.frequency = 0;
    ThreePushData.InstantCurrentB = 0;
    ThreePushData.InstantCurrentR = 0;
    ThreePushData.InstantCurrentY = 0;
    ThreePushData.InstantNeutralCurrent = 0;
    ThreePushData.InstantPowerFactorB = 0;
    ThreePushData.InstantPowerFactorR = 0;
    ThreePushData.InstantPowerFactorThree = 0;
    ThreePushData.InstantPowerFactorY = 0;
    ThreePushData.InstantVoltageB = 0;
    ThreePushData.InstantVoltageR = 0;
    ThreePushData.InstantVoltageY = 0;
    ThreePushData.LSActiveEnergy = 0;
    ThreePushData.LSApparentEnergy = 0;
    ThreePushData.LSCurrentB = 0;
    ThreePushData.LSCurrentR = 0;
    ThreePushData.LSCurrentY = 0;
    ThreePushData.LSReactLagEnergy = 0;
    ThreePushData.LSReactLeadEnergy = 0;
    ThreePushData.LSVoltageB = 0;
    ThreePushData.LSVoltageR = 0;
    ThreePushData.LSVoltageY = 0;
    ThreePushData.paramSize = 0;
    ThreePushData.prepaidBal = 0;
    ThreePushData.prepaidReferenceID = 0;
    ThreePushData.RFMacAddr = 0;
    ThreePushData.RFParentAddr = 0;
    ThreePushData.RFCoordinatorAddr = 0;
    ThreePushData.SignalStrength = 0;

    // Alert Single Push Data
    memset(AlertSinglePushData.MeterNum,0x00,6); // Meter number
    memcpy(AlertSinglePushData.PushDataObis,"0.0.0.0.0.0",11); // Meter number

    tempHH = 0; // Hour
    tempMM = 0; // Min
    tempSS = 0; // Sec
    tempYear = 2022; // year
    tempMon = 1; // month
    tempDay = 1; // Day
    sprintf(tempConvert, "%d", tempYear);
    strcpy(AlertSinglePushData.rtc,tempConvert);
    strcat(AlertSinglePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempMon);
    strcat(AlertSinglePushData.rtc,tempConvert);
    strcat(AlertSinglePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempDay);
    strcat(AlertSinglePushData.rtc,tempConvert);
    strcat(AlertSinglePushData.rtc," ");
    sprintf(tempConvert, "%02ld", tempHH);
    strcat(AlertSinglePushData.rtc,tempConvert);
    strcat(AlertSinglePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempMM);
    strcat(AlertSinglePushData.rtc,tempConvert);
    strcat(AlertSinglePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempSS);
    strcat(AlertSinglePushData.rtc,tempConvert);

    memset(AlertSinglePushData.EswBit,0x00,sizeof(AlertSinglePushData.EswBit)); // Esw bits
    memset(AlertSinglePushData.hexStr,0x00,sizeof(AlertSinglePushData.hexStr)); // Hex bytes
    AlertSinglePushData.RFCoordinatorAddr = 0;
    AlertSinglePushData.EventCode = 0;
    AlertSinglePushData.InstantVoltage = 0;
    AlertSinglePushData.InstantPhaseCurrent = 0;
    AlertSinglePushData.PowerFactor = 0;
    AlertSinglePushData.CumActiveEnergy = 0;
    AlertSinglePushData.totalEventCount = 0;
    AlertSinglePushData.paramSize = 0;

    // Alert Three Push Data
    memset(AlertThreePushData.MeterNum,0x00,6); // Meter number
    memcpy(AlertThreePushData.PushDataObis,"0.0.0.0.0.0",11); // Meter number

    tempHH = 0; // Hour
    tempMM = 0; // Min
    tempSS = 0; // Sec
    tempYear = 2022; // year
    tempMon = 1; // month
    tempDay = 1; // Day
    sprintf(tempConvert, "%d", tempYear);
    strcpy(AlertThreePushData.rtc,tempConvert);
    strcat(AlertThreePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempMon);
    strcat(AlertThreePushData.rtc,tempConvert);
    strcat(AlertThreePushData.rtc,"-");
    sprintf(tempConvert, "%02ld", tempDay);
    strcat(AlertThreePushData.rtc,tempConvert);
    strcat(AlertThreePushData.rtc," ");
    sprintf(tempConvert, "%02ld", tempHH);
    strcat(AlertThreePushData.rtc,tempConvert);
    strcat(AlertThreePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempMM);
    strcat(AlertThreePushData.rtc,tempConvert);
    strcat(AlertThreePushData.rtc,":");
    sprintf(tempConvert, "%02ld", tempSS);
    strcat(AlertThreePushData.rtc,tempConvert);

    memset(AlertThreePushData.EswBit,0x00,sizeof(AlertThreePushData.EswBit)); // Esw bits
    memset(AlertThreePushData.hexStr,0x00,sizeof(AlertThreePushData.hexStr)); // Hex bytes
    AlertThreePushData.RFCoordinatorAddr = 0;
    AlertThreePushData.EventCode = 0;
    AlertThreePushData.InstantCurrentR = 0;
    AlertThreePushData.InstantCurrentY = 0;
    AlertThreePushData.InstantCurrentB = 0;
    AlertThreePushData.InstantVoltageR = 0;
    AlertThreePushData.InstantVoltageY = 0;
    AlertThreePushData.InstantVoltageB = 0;
    AlertThreePushData.InstantPowerFactorR = 0;
    AlertThreePushData.InstantPowerFactorY = 0;
    AlertThreePushData.InstantPowerFactorB = 0;
    AlertThreePushData.CumActiveEnergy = 0;
    AlertThreePushData.totalEventCount = 0;
    AlertThreePushData.paramSize = 0;
}
void HexStringToBitCharStringSingle(uint8_t* input, uint8_t* output)
{
    uint8_t i=0;
    for( i=0; i<ESW_BIT_LEN; i++)
        sprintf(&output[i*2],"%02X", *(input+i));

    for(i=0; i<ESW_BIT_LEN*2; i++)
    {
        switch(AlertSinglePushData.hexStr[i])
        {
        case '0' :
            strcat(AlertSinglePushData.EswBit, "0000");
            break;
        case '1' :
            strcat(AlertSinglePushData.EswBit, "0001");
            break;
        case '2' :
            strcat(AlertSinglePushData.EswBit, "0010");
            break;
        case '3' :
            strcat(AlertSinglePushData.EswBit, "0011");
            break;
        case '4' :
            strcat(AlertSinglePushData.EswBit, "0100");
            break;
        case '5' :
            strcat(AlertSinglePushData.EswBit, "0101");
            break;
        case '6' :
            strcat(AlertSinglePushData.EswBit, "0110");
            break;
        case '7' :
            strcat(AlertSinglePushData.EswBit, "0111");
            break;
        case '8' :
            strcat(AlertSinglePushData.EswBit, "1000");
            break;
        case '9' :
            strcat(AlertSinglePushData.EswBit, "1001");
            break;
        case 'A' :
            strcat(AlertSinglePushData.EswBit, "1010");
            break;
        case 'B' :
            strcat(AlertSinglePushData.EswBit, "1011");
            break;
        case 'C' :
            strcat(AlertSinglePushData.EswBit, "1100");
            break;
        case 'D' :
            strcat(AlertSinglePushData.EswBit, "1101");
            break;
        case 'E' :
            strcat(AlertSinglePushData.EswBit, "1110");
            break;
        case 'F' :
            strcat(AlertSinglePushData.EswBit, "1111");
            break;
        default:
            printf("\n Invalid Hex \n");
            break;
        }
    }
}
void HexStringToBitCharStringThree(uint8_t* input, uint8_t* output)
{
    uint8_t i=0;
    for( i=0; i<ESW_BIT_LEN; i++)
        sprintf(&output[i*2],"%02X", *(input+i));

    for(i=0; i<ESW_BIT_LEN*2; i++)
    {
        switch(AlertThreePushData.hexStr[i])
        {
        case '0' :
            strcat(AlertThreePushData.EswBit, "0000");
            break;
        case '1' :
            strcat(AlertThreePushData.EswBit, "0001");
            break;
        case '2' :
            strcat(AlertThreePushData.EswBit, "0010");
            break;
        case '3' :
            strcat(AlertThreePushData.EswBit, "0011");
            break;
        case '4' :
            strcat(AlertThreePushData.EswBit, "0100");
            break;
        case '5' :
            strcat(AlertThreePushData.EswBit, "0101");
            break;
        case '6' :
            strcat(AlertThreePushData.EswBit, "0110");
            break;
        case '7' :
            strcat(AlertThreePushData.EswBit, "0111");
            break;
        case '8' :
            strcat(AlertThreePushData.EswBit, "1000");
            break;
        case '9' :
            strcat(AlertThreePushData.EswBit, "1001");
            break;
        case 'A' :
            strcat(AlertThreePushData.EswBit, "1010");
            break;
        case 'B' :
            strcat(AlertThreePushData.EswBit, "1011");
            break;
        case 'C' :
            strcat(AlertThreePushData.EswBit, "1100");
            break;
        case 'D' :
            strcat(AlertThreePushData.EswBit, "1101");
            break;
        case 'E' :
            strcat(AlertThreePushData.EswBit, "1110");
            break;
        case 'F' :
            strcat(AlertThreePushData.EswBit, "1111");
            break;
        default:
            printf("\n Invalid Hex \n");
            break;
        }
    }
}
#endif


char* hdlc_ChksumCalculate(unsigned char *pcp,unsigned char len)
{
    unsigned int fcs = 0xFFFF;
    while (len--)
    {
        fcs = (unsigned int)((fcs >> 8) ^ (fcstab[((fcs ^ *(pcp++)) & 0xFF)]));
    }
    *fcsarr = (unsigned char)((fcs & 0x00FF) ^ 0xFF);
    *(fcsarr+1) = (unsigned char)(((fcs & 0xFF00)>>8) ^ 0xFF);
    return fcsarr;
}



int getVal(unsigned char c)
{
    //printf(" %d\n",c);
    int rtVal = 0;
    if(c>='0' && c<='9')
    {
        rtVal = c - '0';
    }
    else if(c>='a' && c<='z')
    {
        rtVal = c - 'a' + 10;
    }
    else
    {
        rtVal = c - 'A' + 10;
    }
    //printf("RT value %X\n",rtVal);
    return rtVal;

}

#ifdef AppendMacFromArray
unsigned int get_index(unsigned int val_lapa)
{
    if(val_lapa <= 0xFF)
    {
        return ((val_lapa & (unsigned int)0xFE) >> 1);
    }
    else if(val_lapa <= 0xFEFF)
    {
        return (((val_lapa & (unsigned int)0xFE) >> 1)|((val_lapa & (unsigned int)0xFE00) >> 2));
    }
}

unsigned int get_lapa(unsigned int index)
{

    if(index < 0x80)
    {
        return ((index << 1)|(unsigned int)1);
    }
    else if(index < 0x4000)
    {
        return (((index & (unsigned int)0x3F80) << 2) | ((index & (unsigned int)0x7F) << 1) | 1);
    }
}

int get_hex_to_decimal(char hexn, int base_index)
{
    return ((hexn&0xF0)>>4)*(int)(pow(16,base_index+1)) + (hexn&0x0F)*(int)(pow(16,base_index));
}
#endif

void meterInfo(unsigned char *rbuff)
{

    //printf("Entering Metermac\n");

    FILE *meterInfo;
    unsigned char c[60];

#ifdef ENABLE_COMPARE_MAC
    unsigned char meterMacID[3];
#endif
    unsigned char meterSerialNo[60];

    int eofFlag=0;
    meterInfo = fopen(METERINFO_FILE_PATH,"a+");

#ifdef AppendMacFromArray_ssc
    uint8_t matched_id = 0;
    for(int i=1; i<LAPA_MAC_ARRAY_SIZE; i++)
    {
        for(int k=0; k<8; k++)
        {
            //printf("%c",ARRAY_LAPA_MID[i][k]);
            if(ARRAY_LAPA_MID[i][k] != *(rbuff+26+k))
            {
                matched_id = 0;
                break;
            }
            else
            {
                matched_id = 1;
            }
        }

        if(matched_id == 1)
        {
            printf("\n Repeted Meter Id Found SO fail\n");
            eofFlag = 1;
            break;
        }
    }

#else

    int flag=0;

    //printf("checking meterinfo.txt\n");

    while(fgets(c, 60, meterInfo))
    {

        for(int i=42; i<50; i++)
        {
            meterSerialNo[i] = c[i];
        }

#ifdef ENABLE_COMPARE_MAC

        for(int i=0; i<3; i++)
        {

            meterMacID[i] = getVal(c[35+(i*2)])*16;
            meterMacID[i] += getVal(c[36+(i*2)]);
#endif

        if((meterSerialNo[45] == *(rbuff+29))&&(meterSerialNo[46]== *(rbuff+30))&&(meterSerialNo[47]== *(rbuff+31))&&(meterSerialNo[48]== *(rbuff+32))&&(meterSerialNo[49]== *(rbuff+33)))
        {

            //printf("\n push port = %d \n\n",PushRespPort);
            //printf("\n ssh_port port = %d \n\n",ssh_port);

#ifdef ENABLE_COMPARE_MAC


            for(int i=0; i<3; i++)
            {
                if(meterMacID[i] != *(rbuff+22+i))
                {
                    //that means MAC DOESN't matched so new entry
                    flag = 1;
                    break;
                }
                else
                {
                    flag = 0;
                }
            }

            if(flag != 1)
            {
                //it means mac matched
                printf("\n Repeted Meter Id Found SO fail \n");
                eofFlag = 1;
                break;
            }
            else
            {
                //printf("\n May be duplicate Entry \n");
            }

#else
            printf("\n Repeted Meter Id Found SO fail\n");
            eofFlag = 1;
            break;
#endif

        }
    }

#ifdef ENABLE_COMPARE_MAC
    if(flag == 1)
    {

        printf("\n Duplicate Entry FOUND meterID: ");
        for(int i=26; i<34; i++)
        {
            printf("%c",*(rbuff+i));
        }
        printf("\n");

        eofFlag = 0;
    }
#endif

#endif

    if(eofFlag == 0)
    {
        //printf("adding meter into meterinfo.txt\n");

        for(int i=0; i<8; i++)
        {
            fprintf(meterInfo,"%c",'0');
        }

        fprintf(meterInfo,"%c",' ');

        for(int i=9; i<25; i++)
        {
            fprintf(meterInfo,"%02X",*(rbuff+i) );
        }

        fprintf(meterInfo,"%c",' ');

        for(int i=26; i<34; i++)
        {
            fprintf(meterInfo,"%c",*(rbuff+i) );
        }

        fprintf(meterInfo,"%s","\n");

        //printf("ADDED METER SUCCESSFULLY\n");

    }

    fclose(meterInfo);


}

void appendMac(unsigned char *rbuff, int location, int nodeAddr)
{
#ifdef AppendMacFromArray
    //fetch lapa from rbuff
    unsigned int lapa = get_hex_to_decimal((rbuff[location+4]),4) + get_hex_to_decimal((rbuff[location+5]),2) + get_hex_to_decimal((rbuff[location+6]),0);

#ifdef ENABLE_PUSHRESPONSE_PI
    if(lapa == 0)
    {
        //we assume here this is only possible when we are appending mac in pushresponse to be sent directly from pi
        // it means it is a push reponse packet
        // we had already added the mac in the push response packet so just return
        return;
    }
#endif

    unsigned int index = get_index(lapa);
    if(index > (LAPA_MAC_ARRAY_SIZE -1))
    {
        printf(" LAPA: %u %x, index: %u %x how it is possible !!!\n",lapa,lapa,index,index);

#ifdef SSC_CURL

        char data[200] = {'\0'};

        //printf("\n MeterMac Push   = %s \n",temp_val_FirstHop);
        sprintf(data,
                "&dcuId=%d"
                "&lapa=%02X"
                "&error=%s",
                dcu_id,lapa,"Incorrect LAPA");

        perform_curl_operation_error(data,strlen(data));

#endif
        return;

    }
    memcpy(rbuff+nodeAddr,&ARRAY_LAPA_MAC[index][0], 16);

#ifdef Debug_Print_Enable_LAPA
    printf("\nAPPENDMAC LAPA: %u %x index: %u %x MAC: ", lapa,lapa,index, index);
    for(int k=0; k<16; k++)
    {
        printf("%x", ARRAY_LAPA_MAC[index][k]);

    }

    if((ARRAY_LAPA_MAC[index][0]==0) && (ARRAY_LAPA_MAC[index][1]==0) && (ARRAY_LAPA_MAC[index][2]==0) && (ARRAY_LAPA_MAC[index][3]==0))
    {
        printf("\n METER MAC IS NULL CHECK METERINFO FILE \n \n");


        char temp1[200] = {'\0'};

        //printf("\n MeterMac Push   = %s \n",temp_val_FirstHop);
        sprintf(temp1,
                "&dcuId=%d"
                "&lapa=%02X"
                "&error=%s",
                dcu_id,lapa,"MAC NOT FOUND IN FILE");

        perform_curl_operation_error(temp1,strlen(temp1));


#else
    FILE *meterMac;
    unsigned char c[200];
    int physicalLogicalId[4];
    int macAddr[16];

    meterMac = fopen(METERINFO_FILE_PATH,"r");

    while(fgets(c, 200, meterMac))
    {


        physicalLogicalId[1] = getVal(c[0])*16;
        physicalLogicalId[1] += getVal(c[1]);

        physicalLogicalId[0] = getVal(c[2])*16;
        physicalLogicalId[0] += getVal(c[3]);

        physicalLogicalId[3] = getVal(c[4])*16;
        physicalLogicalId[3] += getVal(c[5]);

        physicalLogicalId[2] = getVal(c[6])*16;
        physicalLogicalId[2] += getVal(c[7]);


        for(int i=0; i<16; i++)
        {
            macAddr[i] = getVal(c[9+(i*2)])*16;
            macAddr[i] += getVal(c[10+(i*2)]);


        }


        if((rbuff[location+3]==physicalLogicalId[1])&&(rbuff[location+4]==physicalLogicalId[0])&&(rbuff[location+5]==physicalLogicalId[3])&&(rbuff[location+6]==physicalLogicalId[2]))
        {
            for(int i=0; i<16; i++)
            {
                rbuff[nodeAddr+(i*2)] = macAddr[0+(i*2)];
                rbuff[nodeAddr+1+(i*2)] = macAddr[1+(i*2)];
            }

            break;
        }

    }
    fclose(meterMac);
#endif
}

//This fuction is used to insert the data coming from HES to Queue and this queue we need to write data to dcu from writeTh() function.
//socket_id is of no use here
//framelength : how much byte we should kept,i.e depends upon the different request.

void insertPollRequest( int socket_id,int frameLength,char *msg)
{

    for(int i=0; i<frameLength; i++)
    {
        message.mesg_text[i]=*(msg+i);
    }

#ifdef Enable_Single_Thread_MSGQ
    message.mesg_type = 1;
#else
    message.mesg_type = 2;
#endif
    msgsnd(msgid, &message, sizeof(message), 0);

}

//This fuction is used to insert the data coming from HES to Queue and this queue we need to write data to dcu from writeTh() function.
//socket_id is of no use here
//framelength : how much byte we should kept,i.e depends upon the different request.

void insert_PushResponse( int socket_id,int frameLength,char *msg)
{

    for(int i=0; i<frameLength; i++)
    {
        message.mesg_text[i]=*(msg+i);
    }
    message.mesg_type = 1;
    msgsnd(msgid, &message, sizeof(message), 0);

}


//This function checks 7E as first byte .If yes then it forward the data to uart and discard the other data i.e invalid data.
//This fuction write data from PI to DCU.
int send_to_BBB(char* message, int length)
{

    if(!BBB_Socket_Disconnect)
    {
        if(send(BBB_fd,message,length,MSG_NOSIGNAL)<0)
        {
            perror("Failed to send BBB SEND ERROR");
            return 0;
        }
        else
        {
            printf("sent to BBB sucessfully\n");
            return length;
        }
    }
    else
    {
        printf("BBB SOCKET NOT CONNECTED CANNOT SEND DATA\n");
    }
}


void* Msg_Queue_writeTh()
{
    while(1)
    {
        msgrcv(msgid, &message, sizeof(message), 1, 0);

        //append mac on getting data from queue
        appendMac(message.mesg_text,0,134); //0: request index 134: 150-16
        return_writhTH = send_to_BBB(message.mesg_text,150);
        memset(message.mesg_text,'\0',150);
    }

}

void* writeTh(int message_type)
{
    usleep(1000);

    msgrcv(msgid, &message, sizeof(message), message_type, 0);

    return_writhTH = send_to_BBB(message.mesg_text,150);
    printf("\n 1-return_writhTH = %d \n",return_writhTH);
    usleep(1000);
    memset(message.mesg_text,'\0',150);

}




void send_pushResponse_from_PI(int dataLen,unsigned char* meterMac)
{
    unsigned char rbuff[150] = {'\0'};
    int nodeAddr = 134; //150-16
    printf("\nsend_pushResponse_from_PI\n");
    //printf(":dataLen = %x %d\n", dataLen, dataLen);
    switch(dataLen)
    {
    case PUSHRESPONSE_INTERVAL_SIZE: //datalen = 92
        memcpy(rbuff,pushResponse_interval,sizeof(pushResponse_interval));
        break;

    case PUSHRESPONSE_NEW_SINGLE_TAMPER_SIZE: // 8B
    case PUSHRESPONSE_NEW_THREE_TAMPER_SIZE: // 7B
    case PUSHRESPONSE_TAMPER_SIZE: //datalen = 68

        memcpy(rbuff,pushResponse_tamper,sizeof(pushResponse_tamper));
        break;

    case PUSHRESPONSE_TAMPER_SIZE_OLD: //datalen = 65
        memcpy(rbuff,pushResponse_tamper,sizeof(pushResponse_tamper));
        break;

    default: //others
        memcpy(rbuff,pushResponse_interval,sizeof(pushResponse_interval));
        break;
    }

    //copy the mac address taken from push packet
    memcpy(rbuff+nodeAddr,meterMac+9,16);

    //insert the push reponse on the message queue
    insert_PushResponse(0,150,rbuff);

}


void* readTh_BBB()
{

    int pushfd;
    int status=0, response_type = 0, pushSocketerror = 0, push_socketDisconnect=1;
    unsigned char meterMac[34]= {0};
    //creating socket BBB
    while(1)
    {
        BBB_fd = socket(AF_INET,SOCK_STREAM,0);
        if(BBB_fd < 0)
        {
            perror("BBB socket error");
        }
        else
        {
            printf("BBB SOCKET CREATED SUCCESSFULLY\n");
            break;
        }
    }

    //creating socket push
    while(1)
    {
        pushfd = socket(AF_INET,SOCK_STREAM,0);
        if(pushfd < 0)
        {
            perror("push socket error");
        }
        else
        {
            printf("push SOCKET CREATED SUCCESSFULLY\n");
            break;
        }
    }

    //connecting to BBB_PORT  as client
    struct sockaddr_in Addr;
    int len=sizeof(Addr);
    Addr.sin_family=AF_INET;
    Addr.sin_port=htons(BBB_PORT);//atoi(argv[1]));
    Addr.sin_addr.s_addr=INADDR_ANY;//argv[2]);

    if(connect(BBB_fd,(const struct sockaddr*)&Addr,len)<0)
    {
        perror("connect to BBB socket error");
        BBB_Socket_Disconnect = 1;
        close(BBB_fd);
    }
    else
    {
        BBB_Socket_Disconnect = 0; //successfull connection
    }

    //connecting to push socket  as client
    struct sockaddr_in Addrp;
    int lenp=sizeof(Addrp);
    Addrp.sin_family=AF_INET;
    Addrp.sin_port=htons(pushPort);//atoi(argv[1]));
    Addrp.sin_addr.s_addr=inet_addr(pushipAddr);//argv[2]);

    if(connect(pushfd,(const struct sockaddr*)&Addrp,lenp)<0)
    {
        perror("connect to push socket error");
        push_socketDisconnect = 1;
        close(pushfd);
    }
    else
    {
        push_socketDisconnect = 0; //successfull connection
    }


    while(1)
    {

        if(BBB_Socket_Disconnect == 1)
        {
            //create socket again
            BBB_fd = socket(AF_INET,SOCK_STREAM,0);
            //try to connect again
            if(connect(BBB_fd,(const struct sockaddr*)&Addr,len)<0)
            {
                perror("connect to BBB socket error");
                BBB_Socket_Disconnect = 1;
                close(BBB_fd);
                //sleep(BBB_TIMEOUT);
                exit(0);
                continue;
            }
            else
            {
                BBB_Socket_Disconnect = 0; //successfull connection
            }

        }

        if(push_socketDisconnect == 1)
        {
            //create socket again
            pushfd = socket(AF_INET,SOCK_STREAM,0);
            //try to connect again
            if(connect(pushfd,(const struct sockaddr*)&Addrp,lenp)<0)
            {
                perror("connect to push socket error");
                push_socketDisconnect = 1;
                close(pushfd);
                sleep(5);
                continue;
            }
            else
            {
                push_socketDisconnect = 0; //successfull connection
            }

        }

        while(1)
        {
            //reset the buffer
            memset(BBB_rbuff,0,BBB_BUFFER_SIZE);

            //reset response type
            response_type = 0;

            //we will try to recv some data
            status=recv(BBB_fd,BBB_rbuff,BBB_BUFFER_SIZE,0);

            //check status condition
            if(status < 0)
            {
                //it means bad data
                perror("BBB recv failed");
                printf("status = %d ",status);
                BBB_Socket_Disconnect = 1;
                close(BBB_fd);
                break; //data is of no use
            }
            else if(status == 0)
            {
                //it means socket is closed but data can be useful
                perror("BBB recv socket close ");
                printf("status = %d ",status);
                BBB_Socket_Disconnect = 1;
                close(BBB_fd);
            }
            else
            {
                //data is useful
                BBB_Socket_Disconnect = 0;
            }

            for(int g=0; g<150; g++)
            {

            }

            int datalen = BBB_rbuff[2];
            if((BBB_rbuff[0] == 'N') && (BBB_rbuff[1] == 'R'))
            {
                response_type = BBB_NR_RESPONSE;
                NR = ((getVal(BBB_rbuff[2]))<<12 | (getVal(BBB_rbuff[3]))<<8 | (getVal(BBB_rbuff[4]))<<4 | (getVal(BBB_rbuff[5])));
            }

            //checking data length validity
            else if((BBB_rbuff[0] != 0x7E) || (BBB_rbuff[datalen+1] != 0x7E) || (BBB_rbuff[1] != 0xA0))
            {
                //it is unknown or incorrect response
                response_type = BBB_RESPONSE_UNKNOWN;

                printf("Incorrect Response to PI\n");
                printf("%X %X %X \n",BBB_rbuff[0],BBB_rbuff[datalen+1],BBB_rbuff[1]);
            }
            //checking if it is push data by checking FE FE FE FE
            else if((BBB_rbuff[3]==0xFE)&&(BBB_rbuff[4]==0xFE)&&(BBB_rbuff[5]==0xFE)&&(BBB_rbuff[6]==0xFE))
            {
                //it is a push response
                response_type = BBB_PUSH_RESPONSE;


                //print the reponse as needed
                printf("\n Meter ID = ");
                int m=0;
                for(int k=17; k<25; k++)
                {
                    printf("%C",BBB_rbuff[k]);
                    meterMac[k+9] = BBB_rbuff[k];
                    meterTitle[m] = BBB_rbuff[k];
                    m++;
                }
                for(int k=0; k<16; k++)
                {
                    //printf("%C",BBB_rbuff[2+dataLen+k]);
                    meterMac[k+9] = BBB_rbuff[2+datalen+k];
                }
                meterMac[8] = meterMac[25] = ' ';

                printf("\n");
                printf("\t");


                //	for(int k=17;k<25;k++)
                //	printf("%X ",BBB_rbuff[k]);
                //printf("\n");

#ifdef Enable_NodeList_Info
                printf("\n SSC RF INFO = ");
                for(int k=0; k<31; k++)
                {
                    printf("%C", BBB_rbuff[2+datalen+16+k]);
                }
                printf("\n");

#endif
                NR = getVal(BBB_rbuff[2+datalen+16+3])*100 + getVal(BBB_rbuff[2+datalen+16+4])*10 +  getVal(BBB_rbuff[2+datalen+16+5]);
                meterInfo(meterMac);
                push_count++;

#ifdef Enable_Dcu_Reset
                //reset the Dcu_reset timer
                prev_push_time = clock();
#endif


            }
            else
            {
                //it is a poll reponse as incorrect response is already tested
                response_type = BBB_POLL_RESPONSE;

                printf("\n POLL RESPONSE CAME FROM METER NODE TO PI VIA DCU  LAPA = %02X %02X %02X %02X \n",BBB_rbuff[4],BBB_rbuff[5],BBB_rbuff[6],BBB_rbuff[7]);

            }

            //it's time to send the response

            switch(response_type)
            {
            case BBB_POLL_RESPONSE:
            {
                if(BBB_rbuff[4]==0x00 && BBB_rbuff[5]==0x02 && BBB_rbuff[6]==0x00 && BBB_rbuff[7]==0x03)
                {
                    printf("\n BBB SNRM SO SKIP \n");
                    break;
                }

                int i_poll=0;
                int poll_fd;

                for(int i_poll=0; i_poll<threadCount; i_poll++)
                {
                    if((meterTag[(i_poll*4)+0] == BBB_rbuff[4])&&(meterTag[(i_poll*4)+1] == BBB_rbuff[5])&&(meterTag[(i_poll*4)+2] == BBB_rbuff[6])&&(meterTag[(i_poll*4)+3] == BBB_rbuff[7]))
                    {
                        // poll_fd = HEScfd[i_poll];
                        poll_fd = HEScfd[0];
                        break;

                    }
                }

                if(send(poll_fd,BBB_rbuff,datalen+2,MSG_NOSIGNAL)<0)
                {
                    perror("send poll response error:");
                    printf("\n\n\n\n Socket error = %d \n\n\n",poll_fd);

                }
                else
                {

                    printf("\n POLL RESPONSE SENT SUCCESSFULLY FROM PI TO HES \n \n \n");
                }

                break;

            }

            case BBB_RESPONSE_UNKNOWN:
            {
                int i_pollu=0;
                int poll_ufd;

                for(int i_pollu=0; i_pollu<threadCount; i_pollu++)
                {
                    if((meterTag[(i_pollu*4)+0] == BBB_rbuff[4])&&(meterTag[(i_pollu*4)+1] == BBB_rbuff[5])&&(meterTag[(i_pollu*4)+2] == BBB_rbuff[6])&&(meterTag[(i_pollu*4)+3] == BBB_rbuff[7]))
                    {
                        //ssc poll_ufd = HEScfd[i_pollu];
                        poll_ufd = HEScfd[0];

                        break;

                    }
                }

                printf("\nIncorrect Response From DCU to PI\n");
                //ssp printf("\n *************1fd = %d**************************** \n",fd);

                if(send(poll_ufd,response_buffer,datalen+2,MSG_NOSIGNAL)<0)
                {
                    perror("send incorrect poll response error:");
                    printf("\n\n\n\n Socket error = %d \n\n\n",poll_ufd);
                }

                break;
            }

            case BBB_PUSH_RESPONSE:
            {
                int pushdataindex;
                hdlc_ChksumCalculate(BBB_rbuff+1,datalen-2);		//ssc //calculate checksum here.As in meter the checksum is calculated like :
                //7E A0 len ....7X chk1 chk2 7E.Checksum is calculated for A0 to len-2 i.e A0 to 7X.

                if((fcsarr[0]==BBB_rbuff[datalen-1])&&(fcsarr[1]==BBB_rbuff[datalen]))
                {
                    //pushdataindex = BBB_rbuff[14]+8;				//ssc As data is encrypted/wrapper from location after FE FE FE i.e response_buffer+7 .pushdataindex is wrapper frame length.
                    BBB_rbuff[datalen+2] = dcu_id_push[0];  //MSB
                    BBB_rbuff[datalen+3] = dcu_id_push[1];  //LSB
                    printf("\n ***************check sum Matched****************************** \n");

                    char data[500] = {'\0'};

                    char temp_val_destNode[17] = {0};
                    char temp_val_FirstHop[17] = {0};
                    int j=0;
                    for(int m=17; m<25; m++)
                    {
                        sprintf(&temp_val_destNode[j], "%C",meterMac[m+9] );//%.4x
                        j++;
                    }

                    //printf("\n MeterClientID  = %s \n",temp_val_destNode);
                    j=0;

                    //PRINTF("\n temp_val_FirstHop val = %s \n",temp_val_FirstHop);
                    for(int s=8; s<16; s++)
                    {

                        sprintf(&temp_val_FirstHop[j], "%02X",meterMac[s+9] );
                        j=j+2;
                    }


                    //printf("\n MeterMac Push   = %s \n",temp_val_FirstHop);
                    j=0;
                    sprintf(data,
                            "&dcuId=%d"
                            "&destNode=%s"
                            "&firstHop=%s"
                            "&routeEvent=%s",
                            dcu_id,temp_val_destNode,temp_val_FirstHop,"PUSH");

                    perform_curl_operation(data,strlen(data));


                    printf("\n ***************DecryptPush************************** \n");
                    DecryptPush(BBB_rbuff, (datalen+2));

                    // if(send(pushfd,BBB_rbuff+7,(pushdataindex),MSG_NOSIGNAL)<0)
                    if(send(pushfd,BBB_rbuff,datalen+2+2,MSG_NOSIGNAL)<0)
                    {
                        //perror("send to push socket");
                        printf("\n ***************send to push socket Error***************************** \n");

                        pushSocketerror++;								//ssc Just to check if hes jar is stopped/crashed so that the connection resumes itself.
                        close(pushfd);

                        pushfd=socket(AF_INET,SOCK_STREAM,0);
                        if(connect(pushfd,(const struct sockaddr*)&Addrp,lenp)<0)
                        {
                            perror("connect to push socket");//return 0;
                            push_socketDisconnect = 1;
                            close(pushfd);
                        }
                        if(pushSocketerror==20)
                        {
                            printf("Not able to make connection again to push socket");
                            pushSocketerror=0;
                            push_socketDisconnect = 1;
                        }
                    }
                    else
                    {
                        //push sent successfully now need to send push reponse if macro enabled
                        printf("\n \n push sent successfully \n");
                        printf("pushdataindex: %d , datalen: %d\n", pushdataindex, datalen);
                        push_socketDisconnect = 0;

                        send_pushResponse_from_PI(datalen,meterMac);
                    }


                }

                break;
            }


            }


            if((status == 0) || (push_socketDisconnect == 1))
            {

                break;
            }

        }

    }


}


#ifdef Enable_Dcu_Reset

/*DCU RESET Thread*/
void* dcu_reset_thread()
{
    while(1)
    {
        sleep(PUSH_INTERVAL);
        clock_t current_time = clock();
        if(((double)(current_time - prev_push_time)/CLOCKS_PER_SEC) >= PUSH_INTERVAL )
        {
            //we need to reset the DCU and log the reset in nohup.out

            printf("\n*********************** PYTHON RESET  RF CALLED ************************\n");
            system("nohup python resetRF_new.py &");


        }
    }
}
#endif


/*///////////////////////////////////////////////////////////////////////

#Working  : This fuction binds and listen the socket
#Arg      : Arg1 Port no and Arg2:socket id


///////////////////////////////////////////////////////////////////////*/


int hesSocket(int HESPort,int sfd)
{
    //puts("Binding...");
    struct sockaddr_in sAddr;
    int slen=sizeof(sAddr);
    sAddr.sin_family=AF_INET;
    sAddr.sin_port=htons(HESPort);
    sAddr.sin_addr.s_addr=INADDR_ANY;

    //ssc //This function was called becasue there was some error in binding of socket,port was geting closed sometimes.
    if((setsockopt(sfd,SOL_SOCKET, SO_REUSEADDR, &(int)
{
    1
}, sizeof(int)))<0)
    {
        perror("setsockopt");
        close(sfd);
#ifndef ENABLE_FD_LEAKAGE_DEBUG
        return 0;
#else
        return -1;
#endif
    }


    if(bind(sfd,(const struct sockaddr*)&sAddr,slen)<0)
    {
        perror("bind");
        close(sfd);
#ifndef ENABLE_FD_LEAKAGE_DEBUG
        return 0;
#else
        return -1;
#endif
    }
    //puts("bind success");
    //puts("creating connection queue for HES only");
    if(listen(sfd,1)<0)
    {
        perror("listen");
        close(sfd);
#ifndef ENABLE_FD_LEAKAGE_DEBUG
        return 0;
#else
        return -1;
#endif
    }
    //puts("listen success");

#ifdef ENABLE_FD_LEAKAGE_DEBUG
    return 1;
#endif

    //puts("************waiting And Listening passively for connection request from any client no **********");

}


/*///////////////////////////////////////////////////////////////////////

#Working  : Accept the request coming from HES and write the request in messagequeue.
#Arg nums : Arg is port no.

///////////////////////////////////////////////////////////////////////*/

void* threadWork(void* nums)
{
    int  sfd,pifd,cfd,cPifd,HESdisconnect=0,reqIndex=0,datalen=0;
    struct sockaddr_in cAddr;
    int clen=sizeof(cAddr);
    struct sockaddr_in cPiAddr;
    int cPilen=sizeof(cPiAddr);
    FILE *log;
    char name[100];
    const char* dir = "/home/pi/Downloads/";
    const char* type = ".txt";
    int HESPort = *(int*)(nums);
    unsigned char rbuff[150]= {'\0'};			//ssc //rbuff is request buffer i.e poll request
    int status=0,strLen=0, status1=0;
    int slen;


    /**************************************SOCKET COMMUNICATION START******************************************/
#ifdef ipv6_poll
    sfd = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

    if (sfd == -1)
    {
        perror("socket()");
        exit(EXIT_FAILURE);
    }
    else
        printf("\n LOCAL Socket successfully created..\n");

    if((setsockopt(sfd,SOL_SOCKET, SO_REUSEADDR, &(int)
{
    1
}, sizeof(int)))<0)
    {
        perror("setsockopt");
        close(sfd);

    }
    else
        printf("\n setsockopt Socket successfully created..\n");

    struct sockaddr_in6 sAddr;
    slen=sizeof(sAddr);
    sAddr.sin6_family=AF_INET6;
    sAddr.sin6_port=htons(HESPort);
    sAddr.sin6_addr=in6addr_any;
    if(bind(sfd,(const struct sockaddr*)&sAddr,slen)<0)
    {
        perror("bind");
        close(sfd);

        exit(EXIT_FAILURE);
    }
    else
        printf("\n bind Socket successfully created..\n");

    if(listen(sfd,5)<0)
    {
        perror("listen");
        close(sfd);

        exit(EXIT_FAILURE);

    }
    else
        printf("Listener on port = %d\n", HESPort);

    cfd=accept(sfd,(struct sockaddr*)&cAddr,&clen);

    if (cfd < 0)
    {
        printf(" \n SERVER ACCEPT FAILED...\n");
        perror("accept");
        HESdisconnect = 1;
    }
    else
    {
        printf(" \n SERVER ACCEPTED CLIENT REQUEST...\n");
        HESdisconnect = 0;
    }

    HESsfd[0] = sfd;
    HEScfd[0] = cfd;
#endif





    while(1)
    {
        if(HESdisconnect==1)
        {
            // printf("trying to regain connection with port %d",HESPort);

#ifdef ipv6_poll

            sfd = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

            if (sfd == -1)
            {
                perror("socket()");
                //exit(EXIT_FAILURE);
            }


            if((setsockopt(sfd,SOL_SOCKET, SO_REUSEADDR, &(int)
        {
            1
        }, sizeof(int)))<0)
            {
                perror("setsockopt");
                close(sfd);

            }

            struct sockaddr_in6 sAddr;
            slen=sizeof(sAddr);
            sAddr.sin6_family=AF_INET6;
            sAddr.sin6_port=htons(HESPort);
            sAddr.sin6_addr=in6addr_any;

            if(bind(sfd,(const struct sockaddr*)&sAddr,slen)<0)
            {
                perror("bind");
                close(sfd);

                //exit(EXIT_FAILURE);
            }

            if(listen(sfd,5)<0)
            {
                perror("listen");
                close(sfd);

                //exit(EXIT_FAILURE);

            }

#endif
            cfd=accept(sfd,(struct sockaddr*)&cAddr,&clen);
            if(cfd<0)
            {
                perror("accept");
                printf("\n\n\nHES is not connecting\n\n\n");

#ifdef ENABLE_FD_LEAKAGE_DEBUG
                HESdisconnect=1; //rk making sure it remains 1
#endif

                continue;
            }
            // puts("***********************connection accepted from HES*************************");
            HESdisconnect = 0;


        }

        while(1)
        {
            memset(rbuff,'\0',150);
            //printf("\n waiting for Poll Request from HES in while(1) ssc1 \n\n\n\n");

            status=recv(cfd,rbuff,150,0);

            //ssc //if connection of HES is lost or none of byte is received then recv return 0 or <0
#ifndef ENABLE_FD_LEAKAGE_DEBUG
            // printf("%s: %d status: %d", __FUNCTION__, HESPort, status);
            if(status<=0)
            {
                HESdisconnect=1;
                close(cfd);
                close(sfd);
                break;
            }
#else
            if(status<0)
            {
                //error in data reception so no need to go further let's start from accept protocol again
                HESdisconnect=1;
                break;
            }
            else if(status == 0)
            {
                //we got some valid data but the port is closed from HES side so we will start from accept protocol once we have the data processed
                HESdisconnect=1;
                close(cfd);	//close the cfd as it is already in closed_wait state
            }
            else
            {
                HESdisconnect=0; //data comes successfully without any errors
            }
#endif

#if defined(Enable_Single_Thread_MSGQ) && !defined(ENABLE_PUSHRESPONSE_PI)
            if(HESPort == PushRespPort)
            {


                for(int k=0; k<150; k++)
                {
                    if((rbuff[k]==0x7E)&&(rbuff[k+1]==0xA0))
                    {
                        reqIndex=k;
                        datalen = rbuff[k+2];

                        if(rbuff[datalen+1]==0x7E)
                        {
                            //nodeAddr = 100-16;//datalen+2;
                            nodeAddr = 150-16;//datalen+2;
                        }
                    }
                    if(k >(datalen+2))
                    {
                        //printf("\n HES PUSH RESPONSE  Came above\n ");
                        printf("\n ************************************HES Push Response to Pi LAPA = %02X %02X %02X %02X*************************\n",rbuff[3],rbuff[4],rbuff[5],rbuff[6]);
                        break;
                    }
                }


            }
            else
            {
                for(int k=0; k<150; k++)
                {
                    if((rbuff[k]==0x7E)&&(rbuff[k+1]==0xA0))
                    {
                        reqIndex=k;
                        datalen = rbuff[k+2];

                        for(int i=0; i<threadCount; i++)
                        {
                            if(pthread_equal(tid[i],pthread_self()))	//ssc compare the current thread running so that i can save the logical and physical
                            {
                                meterTag[(i*4)+0] = rbuff[k+3];			//ssc //logical array location 3rd byte
                                meterTag[(i*4)+1] = rbuff[k+4];			//ssc //logical array location 4th byte
                                meterTag[(i*4)+2] = rbuff[k+5];			//ssc //Physical array location 5th byte
                                meterTag[(i*4)+3] = rbuff[k+6];			//ssc //Physical array location 6th byte

                            }
                        }

                        if(rbuff[datalen+1]==0x7E)						//ssc ////finding last 7E of packet.
                        {
                            nodeAddr = 150-16;
                        }
                    }
                    if(k >(datalen+2))									//ssc ////failsafe condn .
                    {

                        //   printf("\n HES Poll Request Came above \n ");
                        printf("\n *************************** HES POLL REQUEST CAME FROM HES LAPA =%02X %02X %02X %02X*****************************\n",rbuff[3],rbuff[4],rbuff[5],rbuff[6]);
                        //ssc3	printf("\n Failsafe condition executed from threadwork function k =%d and datalen=%d\n",k,datalen+2);
                        break;
                    }
                }

            }
#else
            for(int k=0; k<150; k++)
            {
                if((rbuff[k]==0x7E)&&(rbuff[k+1]==0xA0))
                {
                    reqIndex=k;
                    datalen = rbuff[k+2];

                    for(int i=0; i<threadCount; i++)
                    {
                        if((meterTag[(i*4)+0] == rbuff[k+3])&&(meterTag[(i*4)+1] == rbuff[k+4])&&(meterTag[(i*4)+2] == rbuff[k+5])&&(meterTag[(i*4)+3] == rbuff[k+6]))
                        {
                            //	printf("\n  meterTag = %02X %02X %02X %02X \n",meterTag[(i*4)+0],meterTag[(i*4)+1],meterTag[(i*4)+2],meterTag[(i*4)+3]);
                            meterTag[(i*4)+0] = 0;
                            meterTag[(i*4)+1] = 0;
                            meterTag[(i*4)+2] = 0;
                            meterTag[(i*4)+3] = 0;
                        }


                        if(pthread_equal(tid[i],pthread_self()))	//ssc compare the current thread running so that i can save the logical and physical
                        {
                            meterTag[(i*4)+0] = rbuff[k+3];			//ssc //logical array location 3rd byte
                            meterTag[(i*4)+1] = rbuff[k+4];			//ssc //logical array location 4th byte
                            meterTag[(i*4)+2] = rbuff[k+5];			//ssc //Physical array location 5th byte
                            meterTag[(i*4)+3] = rbuff[k+6];			//ssc //Physical array location 6th byte

                        }
                    }

                    if(rbuff[datalen+1]==0x7E)						//ssc ////finding last 7E of packet.
                    {
                        nodeAddr = 150-16;
                    }
                }
                if(k >(datalen+2))									//ssc ////failsafe condn .
                {
                    //   printf("\n HES Poll Request Came above\n ");
                    printf("\n *************************** HES POLL REQUEST CAME FROM HES LAPA =%02X %02X %02X %02X*****************************\n",rbuff[3],rbuff[4],rbuff[5],rbuff[6]);

                    //ssc3	printf("\n Failsafe condition executed from threadwork function k =%d and datalen=%d\n",k,datalen+2);
                    break;
                }
            }

#endif

#ifndef Enable_Single_Thread_MSGQ
            appendMac(rbuff,reqIndex,nodeAddr);
#endif

            if((rbuff[0]==0x7E)&&(rbuff[datalen+1]==0x7E))
            {
                insertPollRequest(cfd,150,rbuff);
#ifndef Enable_Single_Thread_MSGQ
                writeTh(2);
#endif

            }
#ifdef ENABLE_FD_LEAKAGE_DEBUG
            if(status == 0)
            {
                printf(" Reinitializing HES Connection\n");
                break;	//now we have the data we can start the accept protocol
            }
#endif

            if(status == 0)
            {
                printf("status:%d need to close and reopen the port\n",status);
                break;
            }
        }
    }
}

void* pushResponse(void* nums)
{
    int  sfd,pifd,cfd,cPifd,reqIndex=0,datalen=0;
    struct sockaddr_in cAddr;
    int clen=sizeof(cAddr);
    struct sockaddr_in cPiAddr;
    int cPilen=sizeof(cPiAddr);
    printf("\n Function = %s and line = %d \n",__FUNCTION__,__LINE__);
    FILE *log;

    sfd=socket(AF_INET,SOCK_STREAM,0);
    if(sfd<0)
    {
        perror("socket");
        return 0;
    }
    //puts("\n***************************HES Push Resp socket created successfully********************** \n\n");
#ifndef ENABLE_FD_LEAKAGE_DEBUG
    hesSocket(PushRespPort,sfd);
#else
    if(hesSocket(PushRespPort,sfd) < 0)
    {
        perror("hessocket error");
        return 0; //rk unable to listen on hes socket so we should return, sfd is already closed in the hesScoket function
    }
#endif

    cfd=accept(sfd,(struct sockaddr*)&cAddr,&clen);
#ifndef ENABLE_FD_LEAKAGE_DEBUG
    if(cfd<0)
    {
        perror("accept");
        close(sfd);//return 0;
    }
    //printf("\n***********************Connection accepted from HES from PushRespPort = %d  **********************\n",PushRespPort);
    Push_HesDisconnect =0;
#else
    if(cfd<0)
    {
        perror("accept");
        //close(sfd); 			//rk don't close the sfd as we can reinitialize the accept protocol. It's not logical to close already created sfd.
        Push_HesDisconnect = 1;	//we will make sure that we retry in the while loop
    }
    else
    {
        Push_HesDisconnect = 0; //successfull in accepting the connection
    }
#endif

    unsigned char wbuff[1000]= {'\0'},rbuff[1000]= {'\0'},reply[1000]= {'\0'};
    int status=0,strLen=0, status1=0;

    while(1)
    {
        if(Push_HesDisconnect==1)
        {
            cfd=accept(sfd,(struct sockaddr*)&cAddr,&clen);
            if(cfd<0)
            {
                perror("accept");
                printf("\n\n\nHES is not connecting\n\n\n");
#ifdef ENABLE_FD_LEAKAGE_DEBUG
                Push_HesDisconnect = 1;
#endif
                continue;
            }
            puts("******PUSH HES DISCONNECT *****connection accepted from Push response Port*****************");
            Push_HesDisconnect = 0;

        }

        while(1)
        {
            memset(rbuff,'\0',500);
            //puts("\n **********************WAITING FOR PUSH RESPONSE NOW*****************\n");
            status=recv(cfd,rbuff,200,0);
#ifndef ENABLE_FD_LEAKAGE_DEBUG
            if(status<=0)
            {
                printf("Disconnect from PUSH Response Port\n Reinitializing PUSH Response Connection\n");
                Push_HesDisconnect=1;
                close(cfd);
                break;
            }
#else
            if(status<0)
            {
                //rk it means some error occured so data is corrupted so no need to proceed further. Try again from accept protocol
                printf("Disconnect from PUSH Response Port\n Reinitializing PUSH Response Connection\n");
                Push_HesDisconnect=1;
                break;
            }
            else if(status == 0)
            {
                //rk it means the socket has been closed from other end, but may be we recieved correct data, so need to proceed further to check
                printf("HES push response port closed\n");
                Push_HesDisconnect=1;
                close(cfd);	//rk need to close the cfd as it's in closed_wait state
                //break;  //rk we will not do it as need to check what data has came, we will do break later on
            }
            else
            {
                // rk data is fine so let's make sure that Push_HesDisconnect remains 0
                Push_HesDisconnect = 0;
            }
#endif

            printf("\n ***********************HES Push Response to Pi ********************\n");



            for(int k=0; k<150; k++)
            {
#ifdef Enable_PUSH_And_Response_Print
                printf("%02X ",*(rbuff+k));
#endif
                if((rbuff[k]==0x7E)&&(rbuff[k+1]==0xA0))
                {
                    reqIndex=k;
                    datalen = rbuff[k+2];

                    if(rbuff[datalen+1]==0x7E)
                    {
                        //nodeAddr = 100-16;//datalen+2;
                        nodeAddr = 150-16;//datalen+2;
                    }
                }
                if(k >(datalen+2))
                {
                    break;
                }
            }

            appendMac(rbuff,reqIndex,nodeAddr);

            //	printf("\n\n***************************************MUTEX LOCK CALLED************************************\n\n");

            if((rbuff[0]==0x7E)&&(rbuff[datalen+1]==0x7E))
            {
                //	pthread_mutex_lock(&lock);
                //insert(cfd,datalen+4,rbuff);
                //ssc3	insert(cfd,150,rbuff);
                insert_PushResponse(cfd,150,rbuff);
                writeTh(1);
                //ssc3	usleep(1000);
                //			printf("\n\n***************************************MUTEX UNLOCK CALLED************************************\n\n");

                //	pthread_mutex_unlock(&lock);
            }

#ifdef ENABLE_FD_LEAKAGE_DEBUG
            if(status == 0)
            {
                printf("Reinitializing PUSH Response Connection\n");
                break; //rk now that we have read the data we can again go to accept protocol state
            }

#endif


        }
    }
}


#ifdef AppendMacFromArray
void Prefetching_LAPA_with_MAC()
{
    FILE *meterInfo;
    unsigned char c[200];
    meterInfo = fopen(METERINFO_FILE_PATH,"r");
    uint8_t macAddr[16];
    while(fgets(c, 200, meterInfo))
    {

        //printf("%s\n", c);

        if(c[0] == 'F' && c[1] == 'F')
        {
            //printf("useless\n");
        }
        else
        {
            //rkc: need to consider only last 3 nibbles as they can store a maximum of 4096(FFF) value as a lapa
            unsigned int val_lapa = getVal(c[5])*256 + getVal(c[6])*16 + getVal(c[7]) ;
            if(val_lapa != 0)
            {
                unsigned int array_index =get_index(val_lapa) ;
                //printf("val %d %d ", val_lapa, (val_lapa -1)/2);

                //get the macAddress
                for(int i=0; i<16; i++)
                {
                    //	printf("%x ",getVal(c[9+(i*2)]));
                    macAddr[i] = getVal(c[9+(i*2)])*16;
                    macAddr[i] += getVal(c[10+(i*2)]);
                    //	printf("%x ",getVal(c[10+(i*2)]));
                    ARRAY_LAPA_MAC[array_index][i] = macAddr[i]; //storing the macAddress
                    //	printf("%x",ARRAY_LAPA_MAC[array_index][i]);
                }
                //printf("\n");

                char meterId[8];
                for(int i=0; i<8; i++)
                {
                    meterId[i] = c[42+i];
                    ARRAY_LAPA_MID[array_index][i] = meterId[i]; //storing meterId
                    //printf("%c",ARRAY_LAPA_MID[array_index][i]);
                }
                //printf("\n");
            }

        }

    }

    fclose(meterInfo);
}
#endif

void print_arguments(int argc, char **argv)
{
    printf("Arguments = \n");
    for(int i=0; i<argc; i++)
    {
        printf("%s\n",*(argv+i));
    }
}


int nr_push(int num_route, char* meterTitle_params)
{

    uint8_t push[200];
    uint8_t index1=0;
    uint8_t index=0;
    uint32_t data=0;
    uint16_t voltagedata=0;
    uint32_t IrmData=0;
    unsigned char Push_Data_Obis[]= {0x00,0x00,0x19,0x09,0x00,0xFF};
    unsigned char MeterSerialNumber[]= {0x31,0x31,0x31,0x31,0x31,0x31}; // not used if final data at HES side
    errCode=0;

    //int32 dataInt32=0;
    uint8_t pushClientSystemTitle[]= {0x50, 0x5A, 0x45, 0x30, 0x30, 0x30, 0x33, 0x33};
    uint8_t ServerSystemTitle[8]= { 0x50, 0x5A, 0x45, 0x30, 0x30, 0x30, 0x33, 0x33};
    // unsigned char MeterSerialNo[9] ={0x39,0x30,0x30,0x30,0x33,0x33};
    // unsigned char MeterSerialNo[9] ={0x50,0x5A,0x45,0x30,0x33,0x33};
    //  uint8 iv[12] ={0x50, 0x5A, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t Esw[150];
    // uint8 index1=0;
    uint8_t cTemp[200];
    uint16_t pushDataLen=0;
    uint8_t iTemp = 0;
    uint8_t iv[12];
    uint32_t GCMFrameCounter=0;

    uint8_t ssc = 0;
    memcpy(iv,pushClientSystemTitle,8);


    // SSC_CLK = osal_getClock();
    //  SSC_CLK =1614814813;
    //  osal_ConvertUTCTime(tm,SSC_CLK);
    index1=0;

    memset(push,0x00,200);
    push[index++]=0x0F;
    push[index++]=0xD0;
    push[index++]=0x00;
    push[index++]=0x00;
    push[index++]=0x01;
    push[index++] = 0x0C;
//RTC will be kept in 12bytes
    push[index++] = ssc; // HH
    push[index++] = 0x02; // MM
    push[index++] = 0x01; // SS
    push[index++] = 0xFF;
    push[index++] = 0x07; // YY
    push[index++] = 0xE5; // YY
    push[index++] = 0x01; // MM
    push[index++] = 0x01; // DD
    push[index++] = 0xFF; // day of weak
    push[index++] = 0x00;
    push[index++] = 0x00; // DD
    push[index++] = 0x00; // DD

    push[index++]=0x02;
    push[index++]=0x06; //6 counts of data

    push[index++]=  0x09; // octet string
    push[index++]=  0x06; // no. of  parameter
    for(uint8 i=0; i<6; i++)
        push[index++] = MeterSerialNumber[i]; // Meter number only for send

    push[index++]=  0x09; // octet string
    push[index++]=  0x06; // no. of  parameter
    for(uint8 i=0; i<6; i++)
        push[index++] = Push_Data_Obis[i]; // Push Obis
    /*
    data = (uint32_t)dcu_id; // Dcu Id
    push[index++]=  0x12; // unsigned 16
    push[index++]= (data>>8);
    push[index++]= data;
    */
    data = (uint32_t)dcu_id; // Dcu Id
    push[index++]=  0x06; // unsigned 32
    push[index++]= (data>>24);
    push[index++]= (data>>16);
    push[index++]= (data>>8);
    push[index++]= data;

    data = (uint16_t)num_route; // NR value
    push[index++]=  0x12; // unsigned 16
    push[index++]= (data>>8);
    push[index++]= data;

    uint16_t nr_data = data;

    push[index++]=  0x09; // octet string
    push[index++]=  0x08; // no. of  parameter
    for(uint8 i=0; i<8; i++)
        push[index++] = meterTitle_params[i]; // meter title

    data = (uint16_t)push_count; // Server address id
    push[index++]=  0x12; // unsigned 16
    push[index++]= (data>>8);
    push[index++]= data;

    iv[8]=GCMFrameCounter>>24;
    iv[9]=GCMFrameCounter>>16;
    iv[10]=GCMFrameCounter>>8;
    iv[11]=GCMFrameCounter;

    errCode=1;

    enc_dec(push,index,iv,NULL,0,1);

    errCode=0;

    Esw[index1++]=0x00;
    Esw[index1++]=0x01;
    Esw[index1++]=0x00;
    Esw[index1++]=0x01;

    Esw[index1++]=0x00;
    Esw[index1++]=0x40; //len

    Esw[index1++]=0x00;
    Esw[index1++]=0x00;//why 0x82 in debug?
    Esw[index1++]=0xDB;
    Esw[index1++]=0x08;


    for(uint8 i=0; i<8; i++)
        Esw[index1++]=ServerSystemTitle[i];// client

    Esw[index1++]=index+5;//value 77 coming//index =25
    Esw[index1++]=0x20; //index = 26
    Esw[index1++]=GCMFrameCounter>>24;
    Esw[index1++]=GCMFrameCounter>>16;
    Esw[index1++]=GCMFrameCounter>>8;
    Esw[index1++]=GCMFrameCounter;

    memcpy(Esw+index1,Enc_Data_Buffer,index);

    //   if dont want to send data as encrypted memcpy(Esw+index1,push,index);
    //   memcpy(Esw+index1,push,index);
    index1+= index;
    Esw[6]=(index1-8)>>8;
    Esw[7]=index1-8;


    memset(&cTemp, 0x00, sizeof(cTemp)); // cTemp will contain final data except 7E 7E
    cTemp[0] = 0xA0;
    pushDataLen = index1 + 8; // 133(payload) + 5(A0 + 4 byte address) + 1 (len) + 2 (Fcs) = final length incluing FCS
    cTemp[1] = pushDataLen;
    cTemp[2] = 0xFE;
    cTemp[3] = 0xFE;
    cTemp[4] = 0xFE;
    cTemp[5] = 0xFE;

    pushDataLen = index1; // 133(payload)
    for(iTemp=0; (iTemp <= pushDataLen); iTemp++)
        cTemp[iTemp+6] = Esw[iTemp]; // Copy payload into final data

    pushDataLen = index1 + 6; // 133(payload) + 5(A0 + 4 byte address) + 1 (length) = length except 7E 7E and FCS
    hdlc_ChksumCalculate(cTemp,pushDataLen);

    memset(&Finaltxbuf, 0x00, sizeof(Finaltxbuf));
    Finaltxbuf[0] = 0x7E;
    pushDataLen = index1 + 6;  // 133(payload) + 5(A0 + 4 byte address) + 1 (length)
    for(iTemp=0; (iTemp <= pushDataLen); iTemp++)
        Finaltxbuf[iTemp+1] = cTemp[iTemp]; // Copy (payload + A0 + len + FE + FE + FE + FE) into txbuf

    Finaltxbuf[pushDataLen+1] = fcsarr[0];
    Finaltxbuf[pushDataLen+2] = fcsarr[1];
    Finaltxbuf[pushDataLen+3] = 0x7E;

    pushDataLen = index1 + 10; // c5(A0 + 4 byte address) + 1 (len) + 2 (Fcs) + 7E + 7E
    //testpushDataLen = pushDataLen;

    if(send(pushfd,Finaltxbuf,pushDataLen,MSG_NOSIGNAL)<0)
    {
        //perror("send to push socket");
        perror("\n ***************send NR to push socket Error *****************************");

    }
    else
    {
        printf("\n ***************send NR to push successfully %d %u PUSH_COUNT = %d***************************** \n", pushDataLen, nr_data, push_count);
        printf("\n");
        for(int i=0; i<pushDataLen; i++)
        {
            printf("%02X ", Finaltxbuf[i]);
        }
        printf("\n");
    }

    push_count = 0;

    return pushDataLen;
}

void* send_num_route()
{
    while(1)
    {

        sleep(NR_PUSH_TIME);

        //nr_push(NR, meterTitle);
    }
}

int perform_curl_operation(char* data, int len)
{

    CURL *curl ;
    CURLcode res;

    //curl init
    curl = curl_easy_init();


    if(curl == NULL)
    {
        return 0;
    }



    curl_easy_setopt(curl, CURLOPT_URL, STAGING_URL);

    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)len);
    // curl_easy_setopt(curl, CURLOPT_POSTFIELDS, formDataEncoded.c_str());




    res = curl_easy_perform(curl);

    if(res != CURLE_OK)
    {
        fprintf(stderr, "Error: %s\n", curl_easy_strerror(res));
        return 0;
    }

    curl_easy_cleanup(curl);

}


int perform_curl_operation_error(char* data, int len)
{

    CURL *curl ;
    CURLcode res;

    //curl init
    curl = curl_easy_init();


    if(curl == NULL)
    {
        return 0;
    }

    curl_easy_setopt(curl, CURLOPT_URL, CURL_ERROR_URL);

    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)len);
    // curl_easy_setopt(curl, CURLOPT_POSTFIELDS, formDataEncoded.c_str());

    res = curl_easy_perform(curl);

    if(res != CURLE_OK)
    {
        fprintf(stderr, "Error: %s\n", curl_easy_strerror(res));
        return 0;
    }

    curl_easy_cleanup(curl);

}


int perform_curl_operation_SinglePhasepush(char* data, int len)
{

    CURL *curl ;
    CURLcode res;

    //curl init
    curl = curl_easy_init();


    if(curl == NULL)
    {
        return 0;
    }



    curl_easy_setopt(curl, CURLOPT_URL, STAGING_URL_PUSH_SinglePhase);

    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)len);
    // curl_easy_setopt(curl, CURLOPT_POSTFIELDS, formDataEncoded.c_str());




    res = curl_easy_perform(curl);

    if(res != CURLE_OK)
    {
        fprintf(stderr, "Error: %s\n", curl_easy_strerror(res));
        return 0;
    }

    curl_easy_cleanup(curl);

}

int perform_curl_operation_ThreePhasepush(char* data, int len)
{

    CURL *curl ;
    CURLcode res;

    //curl init
    curl = curl_easy_init();


    if(curl == NULL)
    {
        return 0;
    }



    curl_easy_setopt(curl, CURLOPT_URL, STAGING_URL_PUSH_ThreePhase);

    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)len);
    // curl_easy_setopt(curl, CURLOPT_POSTFIELDS, formDataEncoded.c_str());




    res = curl_easy_perform(curl);

    if(res != CURLE_OK)
    {
        fprintf(stderr, "Error: %s\n", curl_easy_strerror(res));
        return 0;
    }

    curl_easy_cleanup(curl);

}

int perform_curl_operation_SinglePhaseAlert(char* data, int len)
{

    CURL *curl ;
    CURLcode res;

    //curl init
    curl = curl_easy_init();


    if(curl == NULL)
    {
        return 0;
    }



    curl_easy_setopt(curl, CURLOPT_URL, STAGING_URL_PUSH_SinglePhaseAlert);

    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)len);
    // curl_easy_setopt(curl, CURLOPT_POSTFIELDS, formDataEncoded.c_str());




    res = curl_easy_perform(curl);

    if(res != CURLE_OK)
    {
        fprintf(stderr, "Error: %s\n", curl_easy_strerror(res));
        return 0;
    }

    curl_easy_cleanup(curl);

}

int perform_curl_operation_ThreePhaseAlert(char* data, int len)
{

    CURL *curl ;
    CURLcode res;

    //curl init
    curl = curl_easy_init();


    if(curl == NULL)
    {
        return 0;
    }



    curl_easy_setopt(curl, CURLOPT_URL, STAGING_URL_PUSH_ThreePhaseAlert);

    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)len);
    // curl_easy_setopt(curl, CURLOPT_POSTFIELDS, formDataEncoded.c_str());




    res = curl_easy_perform(curl);

    if(res != CURLE_OK)
    {
        fprintf(stderr, "Error: %s\n", curl_easy_strerror(res));
        return 0;
    }

    curl_easy_cleanup(curl);

}


int main(int argc,char **argv)
{
	print_arguments(argc, argv);
    Prefetching_LAPA_with_MAC();


    if(argc < 6)
    {
        printf("\nless arguments : need <pushPort> <pushipAddr> <ssh_port_start_addr> <PushRespPort>\n");
        exit(0);
    }
    dcu_id = atoi(argv[5]);
    uint16_t dcu_id_int16 = (uint16_t)dcu_id;
    dcu_id_push[0] = ((dcu_id_int16 & 0xFF00)>>8); //MSB
    dcu_id_push[1] = (dcu_id_int16 & 0x00FF); //lsb
    ssh_port = atoi(argv[3]);
    PushRespPort = atoi(argv[4]);

    threadCount = 25;										//ssc //Total no of threads for poll request
    int i=0;
    for( i = 0; i < threadCount; i++)						//ssc //3025 starting range of port no //use this in command line arg
    {
        nums[i] = ssh_port+i;
    }

#if defined(Enable_Single_Thread_MSGQ) && !defined(ENABLE_PUSHRESPONSE_PI)
    nums[25] = PushRespPort;
#endif

    pushipAddr=*(argv+2);									//ssc //./pushpoll 4059 139.59.43.159// IP

    pushPort = atoi(argv[1]);								//ssc port

    fdUART1 = open("/dev/ttyS0",O_RDWR|O_NOCTTY|O_NDELAY|O_SYNC);
    if(fdUART1==1)
        printf("\n Error in RF Port!");
    else
        printf("RF_path------------>/dev/ttyS0\n\n");

    printf("\n***********************PUSHPOLL SOFTWARE VERSION NUMBER  = %s  **********************\n",VersionNumber);



    struct termios SerialPortSettings;
    tcgetattr(fdUART1,&SerialPortSettings);

    cfsetispeed(&SerialPortSettings,B115200);//B9600);
    cfsetospeed(&SerialPortSettings,B115200);//B9600);
    //SerialPortSettings.c_cflag &= ~(PARENB|PARODD);
    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;
    SerialPortSettings.c_lflag = 0;
    SerialPortSettings.c_oflag = 0;

    SerialPortSettings.c_cflag &=~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
    SerialPortSettings.c_iflag &= ~( ICANON|ECHO | ECHOE | ISIG);

    SerialPortSettings.c_iflag &= ~( INLCR | IGNCR | ICRNL);

    SerialPortSettings.c_oflag &= ~(OPOST);
    SerialPortSettings.c_cc[VMIN] = 0;
    SerialPortSettings.c_cc[VTIME]=20;
    tcflush(fdUART1,TCIFLUSH);

    if((tcsetattr(fdUART1,TCSANOW,&SerialPortSettings))!=0)
    {
        printf("\n ERROR!in Setting RF Port attributes");
        sleep(20);
    }


#if defined(Enable_Single_Thread_MSGQ) && !defined(ENABLE_PUSHRESPONSE_PI)
    for(i=0; i< threadCount+1; i++)
#else
    for(i=0; i< threadCount; i++)
#endif
    {
        pthread_create(&tid[i], NULL, &threadWork, (void*)(nums+(i)));
    }
    pthread_create(&tid[i],NULL,&readTh_BBB,NULL);

#if !defined(Enable_Single_Thread_MSGQ) && !defined(ENABLE_PUSHRESPONSE_PI)
    pthread_create(&tid[++i],NULL,&pushResponse,NULL);							//ssc Push Resp Thread
#endif


    pthread_create(&tid[++i], NULL, &send_num_route, NULL);
    pthread_create(&tid[++i], NULL, &Msg_Queue_writeTh, NULL);

#ifdef Enable_Dcu_Reset
    pthread_create(&tid[++i], NULL, &dcu_reset_thread, NULL);
    prev_push_time = clock();
#endif

    key = ftok("progfile", 65);
    msgid = msgget(key, 0666 | IPC_CREAT);

    while(1)
    {
#if defined(Enable_Single_Thread_MSGQ) && !defined(ENABLE_PUSHRESPONSE_PI)
        for(i=0; i< threadCount+1; i++)
#else
        for(i=0; i< threadCount; i++)
#endif
        {
            if((pthread_kill(tid[i],0))!=0)
            {
                printf("\n\n\nthread  %d\n\n\n",i);
            }
        }
    }
}


