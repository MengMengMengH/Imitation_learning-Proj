
#ifndef WISEGLOVE_H_
#define WISEGLOVE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>

//�������õ��ĳ���
#define GET_SN  			0xA0
#define GET_MANU			0xA1
#define GET_MODEL			0xA2
#define GET_SENSORALL		0xA3
#define GET_STOP			0xA5 
#define GET_RESET			0xAA	 
#define SET_FEEDBACK        0xA7    
#define ZERO_ALL			0xAA	  
#define SENSOR_LIMITED		19

//��Ԫ�ؽṹ�嶨��
typedef struct {
	float w;   
	float x;
	float y;
	float z;
} QUAT;

class Wiseglove {

public:
	Wiseglove();   //���캯��
	~Wiseglove();  //��������
	/* ***************************************************
	*	���ܣ���USBת���ڶ˿�
	*	������char* port,�˿��ַ���,�� /dev/ttyUSB0
	*         int baud,�̶�Ϊ115200
	*	���أ���ȷ����true,����false
	*/
	bool Open(char* port, int baud = 115200);

	/* ***************************************************
	*	���ܣ��ر�USBת���ڶ˿�
	*	��������
	*	���أ���
	*/
	void Close();

	/* ***************************************************
	*	���ܣ������к�,�ִ���DG05S001L
	*	��������
	*	���أ���ȷ����true,����false
	*/
	bool GetSn(char* sn);

	/* ***************************************************
	*	���ܣ����ͺ�,�ִ���WISEGLOVE5 / WG05F3
	*	��������
	*	���أ���ȷ����true,����false
	*/
	bool GetModel(char* model);

	/* ***************************************************
	*	���ܣ�������,�ִ���XINTIAN
	*	��������
	*	���أ���ȷ����true,����false
	*/
	bool GetManu(char* manu);

	/* ***************************************************
	*	���ܣ�����ָ����������
	*	��������
	*	���أ�����
	*/
	unsigned int GetNumOfSensor(void);
	/* ***************************************************
	*	���ܣ����ֱ۴���������
	*	��������
	*	���أ�����
	*/
	unsigned int GetNumOfArm(void);

	/* ***************************************************
	*	���ܣ�����������������
	*	��������
	*	���أ�����
	*/
	unsigned int GetNumOfPressure(void);

	/* ***************************************************
	*	���ܣ����ñ궨ģʽ
	*	������int mode
	*	mode = 0��  �Զ��궨
	*	mode = 	1�� ��ֹ�Զ��궨
	*	���أ���
	*/
	void SetCalibMode(int mode);
	/* ***************************************************
	*	���ܣ��궨��ָ�Ƕ�
	*	��������
	*	���أ���
	*/
	void ResetCalib();

	/* ***************************************************
	*	���ܣ������д�����ֵ
	*	������unsigned short * data�����������ݣ����19��ushort������
	*	���أ�������ݸ����˷��ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�(0)
	*/
	unsigned int GetData(unsigned short* data);
	/* ***************************************************
	*	���ܣ������д�������һ��ֵ����Χ��0��4096��
	*	������unsigned short * data�����������ݣ����19��ushort������
	*	���أ�������ݸ����˷��ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�(0)
	*/
	unsigned int GetScaledData(unsigned short* data);

	/* ***************************************************
	*	���ܣ����������ĽǶ�
	*	������float *angle���������Ƕ����ݣ����19��float������
	*	���أ�������ݸ����˷��ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�(0)
	*/
	unsigned int GetAngle(float* angle);

	/* ***************************************************
	*	���ܣ�����ָѹ����������ԭʼֵ
	*	������int *pressure��������ԭʼ���ݣ����5��int������
	*	���أ�������ݸ����˷��ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�(0)
	*/
	unsigned int GetPressureRaw(unsigned short* pressure);

	/* ***************************************************
	*	���ܣ����궨�ֱ���̬�����Ԫ��
	*	������float *quat��16��float����,�ֱ�Ϊ:����(wxyz),ǰ��(wxyz),���(wxyz),����(wxyz)
	*          ����������Ԫ�ض���3���ֱ۴��������õ��ͺ�������ͬ
	*	���أ�������ݸ����˷��ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�(0)
	*/
	unsigned int GetQuat(float* quat);

	/* ***************************************************
	*	���ܣ����ֱ�ԭʼ��Ԫ��(��궨�޹�)
	*	������float *quat��16��float����,�ֱ�Ϊ:����(wxyz),ǰ��(wxyz),���(wxyz),����(wxyz)
	*          ����������Ԫ�ض���3���ֱ۴��������õ��ͺ�������ͬ
	*	���أ�������ݸ����˷��ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�(0)
	*/
	unsigned int GetQuatOrg(float* quat);

	/* ***************************************************
	*	���ܣ��궨�ֱ���Ԫ��,���ֱ���ֱ����ֱ����,�����Ļʱִ�д˹���
	*	��������
	*	���أ���
	*/
	void ResetQuat();

	/* ***************************************************
	*	���ܣ�����������ֵ����
	*	��������
	*	���أ���
	*/
	void ZeroPressure(); //��������������

	/* ***************************************************
	*	���ܣ�������ָ������ֵ
	*	������fddata��5��unsigned char����,ȡֵ��Χ0-255, 0��ʾ�޷���,255��־�����ǿ��
	*	���أ�������óɹ� true�����򷵻�false
	*/
	bool  SetFeedBack(unsigned char* fddata);
private:
	static void* Writethread(void* arg);//�������ݵ��̺߳���
	static void* Readthread(void* arg); //�������ݵ��̺߳���
private:
	//////////////////////////////////////////////////////////////
	char manu[16];			//������
	char model[16];			//����
	char serial[16];		//���к�
	//////////////////////////////////////////////////////////////
	unsigned int numofsensor;	//��ָ����������
	unsigned int numofarm;	//�ֱ۴���������
	unsigned int numofpressure;	//��������������
	bool  handtype; //����������
	//////////////////////////////////////////////////////////////
	int timestamp;  //�����̸߳�����	
	int data_update; //�û�����ʹ��
	int scaledata_update;
	int angle_update;
	int quat_update;
	int quatorg_update;
	int pressure_update;
	//////////////////////////////////////////////////////////////
	int retrycmd; //��¼û�����ݸ��µĴ���
	unsigned short pdata[SENSOR_LIMITED];//ѹ��������ֵ
	unsigned short sdata[SENSOR_LIMITED];//��ָ������ֵ
	float armquat[16];					 //�ֱ���Ԫ��
	unsigned short calibdata[SENSOR_LIMITED];		//��һ��ֵ��0-2048��
	float sangle[SENSOR_LIMITED];					//�������Ƕ�
	unsigned short min_sdata[SENSOR_LIMITED];        //��������Сֵ
	unsigned short max_sdata[SENSOR_LIMITED];		//���������ֵ
	float min_adj[SENSOR_LIMITED];					//������С��ȥֵ
	float max_adj[SENSOR_LIMITED];					//��������ȥֵ
	float sanglerange[SENSOR_LIMITED];				//���������˶��Ƕȷ�Χ
	int m_Calibmode;								//�궨ģʽ 0 = �Զ��궨 1=�ֶ��궨		
	bool  resetquat;								//�Ƿ�ִ��QUAT�����㷨 ���ʺ�UNITY3D
	//////////////////////////////////////////////////////////////
	//�ֱ۴������궨��Ҫ�õ��ı���
	QUAT Axis_org;
	QUAT sensor_body, sensor_uparm, sensor_forearm, sensor_hand;
	QUAT sensor_body_new, sensor_uparm_new, sensor_forearm_new, sensor_hand_new;
	QUAT sensor_body_adj;
	QUAT sensor_body_offset, sensor_hand_offset, sensor_forearm_offset, sensor_uparm_offset;
	//�궨���������Ԫ��
	QUAT sensor_body_correct_zero, sensor_uparm_correct_zero, sensor_forearm_correct_zero, sensor_hand_correct_zero;
	//////////////////////////////////////////////////////////////
	unsigned char feedbackold[6];
	unsigned char resendfd;

private:
	int fd;//�ļ�������
	void Sendhostcmd(unsigned char cmd, unsigned char dat);
	int  set_port(int baud);
	int set_speci_baud(int fd, int baud);
	void Createthread(void);
	void Waitthread(void);

	QUAT quatmul(QUAT a, QUAT b);
	QUAT quatconj(QUAT a);

	bool iszeropressure;
	pthread_t thread[2];
	pthread_mutex_t mutport;
	pthread_mutex_t mutdata;
};
#endif /* WISEGLOVE_H_ */
