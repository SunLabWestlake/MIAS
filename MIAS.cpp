/*
\file      multi_cam.cpp
\brief     Multiplexed Image Acquisition System
\version   1.1
\date      2021-4-23
\author    Jing Ning @ Westlake university

Multiplexed Image Acquisition System (MIAS)

Support:

	FLIR
	Basler
	HIKVISION
	DaHeng

Log:
V1.1 20201203 Enable trigger mode of HIKVISION cameras, output start timestamp
V1.0 20201203 Support HIKVISION MVS cameras
V0.8 20181201 Multi-thread processing, support Basler, FLIR and Daheng cameras

*/


//#define USE_DaHeng
//#define USE_Basler
//#define USE_FLIR
//#define USE_HIKVISION
//#define TRIGGER_MODE_OFF

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
using namespace std;

#include <Windows.h>
#include <DbgHelp.h>
#pragma comment(lib,"DbgHelp.lib")
#include <direct.h>

#include "opencv2/highgui/highgui.hpp"
#include "json.h"
#pragma warning(disable : 4996)
#pragma comment(lib,"jsoncpp.lib")
#ifdef _DEBUG
#pragma comment(lib,"opencv_world401d.lib")
#else
#pragma comment(lib,"opencv_world401.lib")
#endif
using namespace cv;

#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>
#include <queue>
#pragma comment(lib,"pthreadVC2.lib")

#ifdef USE_DaHeng
#include "GxIAPI.h"
#include "DxImageProc.h"
#pragma comment(lib, "GxIAPI.lib")
#pragma comment(lib, "DxImageProc.lib")
#define GX_VERIFY(emStatus) if (emStatus != GX_STATUS_SUCCESS) return -1;
#endif

#ifdef USE_Basler
#include <pylon/PylonIncludes.h>
#endif

#ifdef USE_FLIR
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#pragma comment(lib, "Spinnaker_v140.lib")
#endif

#ifdef USE_HIKVISION
#include "MvCameraControl.h"
#pragma comment(lib, "MvCameraControl.lib")
#endif

//-------------------------global-------------------------
#define USE_BUFFER 
#define CAM_PER_TYPE 40
#define MSG_QUEUE_NUM CAM_PER_TYPE*4

int g_multi_part = 0;
int g_buffer_frames = 100;
bool g_is_stop = false;
pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_stop_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_stop_cond = PTHREAD_COND_INITIALIZER;
bool g_need_record = false;
bool g_recorded = false;
bool g_output = true;
int g_show_fps = 0; 
Json::Value g_pConfig;
char g_fourcc[5];
char g_only_one_camera[100];
int g_cmd;
pthread_t pid_c, pid_w[MSG_QUEUE_NUM];
int pid_w_f[MSG_QUEUE_NUM] = { 0 };

//-------------------------utils---------------------------
void create_dir(const char* folderPath) {
	CreateDirectory(folderPath, NULL);
}
void remove_dir(const char* folderPath) {
	char g_tmp[1000];
	//sprintf(g_tmp, "/C rmdir /s /q \"%s\"", folderPath);
	//ShellExecuteA(NULL, "open", "cmd.exe", g_tmp, NULL, SW_HIDE);
	sprintf(g_tmp, "rmdir /s /q \"%s\"", folderPath);
	system(g_tmp);
}

char* get_time_str(char *tmp) {
	time_t t = time(0);
	strftime(tmp, 64, "%H:%M:%S",localtime(&t));
	return tmp;
}
char* get_time_str_f(char *tmp) {
	time_t t = time(0);
	strftime(tmp, 64, "%Y%m%d_%H%M%S", localtime(&t));
	return tmp;
}
//#define USE_PERFORMANCE_FREQUENCY

#ifdef _WIN32
#include <Windows.h>
#pragma comment(lib, "winmm.lib")

LARGE_INTEGER  large_interger;
double performance_frequency;
void init_time(){
#ifdef USE_PERFORMANCE_FREQUENCY
	QueryPerformanceFrequency(&large_interger);
	performance_frequency = large_interger.QuadPart;
	//cout << performance_frequency << endl;
#endif
}

#define EPOCHFILETIME   (116444736000000000UL)
uint64_t get_time()
{
#ifdef USE_PERFORMANCE_FREQUENCY
	QueryPerformanceCounter(&large_interger);
	uint64_t c = large_interger.QuadPart;
	return c * 1000000 / performance_frequency;
#else
	//return ::timeGetTime() * 1000;
	FILETIME ft;
	LARGE_INTEGER li;
	int64_t tt = 0;
	GetSystemTimeAsFileTime(&ft);
	li.LowPart = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;
	tt = (li.QuadPart - EPOCHFILETIME) / 10;
	return tt;
#endif
}
#else
#include <sys/time.h>
float get_time()
{
    timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000+tv.tv_usec;
}
#endif

Json::Value read_config(const char* filename) {
	Json::Value config;
	Json::Reader reader;

	FILE* file = fopen(filename, "rb");
	char config_str[12000] = "{}";
	if (!file) {
		cout << "config file not found, use default config." << endl;
	}
	else {
		fseek(file, 0, SEEK_END);
		long file_size = ftell(file);
		rewind(file);
		fread(config_str, 1, file_size, file);
		config_str[file_size] = '\0';
	}

	bool ret = reader.parse(config_str, config);
	if (!ret) {
		cout << "read config failed: " << reader.getFormatedErrorMessages() << endl;
	} else {
		cout << "config:" << endl << config << endl;
		cout << "\ncmd: \n\tc: close preview and start recording\n\ts: show preview\n\tq: quit\n\n";
	}

	return config;
}
Json::Value get_value(Json::Value* pDict, const char* key, Json::Value default_v) {
	if (!pDict)
		return default_v;
	Json::Value ret = pDict->get(key, default_v);
	return ret;
}

class FrameCounter{
public:
	unsigned long frame_seq;
	unsigned long fps;
	uint64_t last_frame_time;
	uint64_t last_period;
	uint64_t last_time;
	uint64_t cur_time;
	unsigned long frame_count;
	uint64_t period; //us
	const char* tag;
	
	FrameCounter(uint64_t _period = 1000000) {
		last_time = 0;
		period = _period;
		last_period = period;
		frame_count = 0;
		fps = 0;
		frame_seq = 0;
	}

	bool update(uint64_t ts) {
		frame_seq++;
		frame_count++;
		cur_time = ts;
		last_frame_time = cur_time;
		if((cur_time - last_time) > period) {
			last_period = cur_time - last_time;
			fps = frame_count;
			frame_count = 0;
			last_time = cur_time;
			return true;
		}
		return false;
	}
	double get_fps() {
		return fps / (last_period / 1000000.0);
	}
	void set_tag(const char *_tag) {
		tag = _tag;
	}
	
	uint64_t last_time_show;
	uint64_t period_show;
	void init_show_timer(uint64_t t) {
		last_time_show = 0;
		period_show = t * 1000;
	}
	bool update_show_timer() {
		uint64_t show_cur_time = get_time();
		if((show_cur_time - last_time_show) > period_show) {
			last_time_show = show_cur_time;
			return true;
		}
		return false;
	}
};

template<typename T>
class BlockingQueue {
private:
	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t _cond = PTHREAD_COND_INITIALIZER;
	std::queue<T> _queue;
public:
	BlockingQueue() {
		pthread_mutex_init(&_mutex, NULL);
		pthread_cond_init(&_cond, NULL);
	}
	~BlockingQueue() {}

	int put(const T &input) {
		pthread_mutex_lock(&_mutex);
		int num = _queue.size();
		if (num > g_buffer_frames) {
			cout << "f";
			pthread_mutex_unlock(&_mutex);
			return -1;
		}
		_queue.push(input);
		//cout << "put size:" << _queue.size() << endl;
		pthread_mutex_unlock(&_mutex);
		pthread_cond_signal(&_cond);
		return num;
	}

	T take() {
		pthread_mutex_lock(&_mutex);
		while (_queue.empty() && !g_is_stop) {
			pthread_cond_wait(&_cond, &_mutex);
		}
		if (g_is_stop) {
			pthread_mutex_unlock(&_mutex);  //bug fixed !!!
			return NULL;
		}
		T front = _queue.front();
		_queue.pop();
		//cout << "take queue: " << _queue.size() << endl;
		pthread_mutex_unlock(&_mutex);
		return front;
	}
	void interrupt() {
		if (g_is_stop) {
			pthread_cond_signal(&_cond);
		}
	}
	size_t size() {
		pthread_mutex_lock(&_mutex);
		int num = _queue.size();
		pthread_mutex_unlock(&_mutex);
		return num;
	}
};

void* write_thread(void *arg);
class CAMERA_INFO;
struct MSG_INFO;
BlockingQueue<MSG_INFO*> g_msg_queue[MSG_QUEUE_NUM];
void process_msg(MSG_INFO* msg, bool need_delete);

struct MSG_INFO
{
	uint8_t *pData;
	CAMERA_INFO* pCam;
	unsigned long frame_seq;
	float fps;
	uint64_t ts;
	char* pszName;
};

void del_msg(MSG_INFO* msg) {
	if (msg->pData) {
		//cout << "meminfo delete" << msg->frame_seq << endl;
		delete[] msg->pData;
		msg->pData = NULL;
		delete msg;
	}
}

bool put_msg(MSG_INFO* msg, int idx) {
	//process_msg(msg, need_delete);
	//return;
	if (g_msg_queue[idx].put(msg) < 0) {
		del_msg(msg);
		return false;
	}
	//cout << "put_msg " << idx << endl;
	return true;
}

class CAMERA_INFO
{
public:
	int image_width;
	int image_height;	
	bool is_color;
	bool is_open = false;
	int idx;
	char serial_no[100];
	bool is_first_time_init;
	bool is_need_record;
	VideoWriter vw;
	FrameCounter fc;
	uint64_t init_ts;
	int uc;
	size_t data_size;
	int lost;

	ofstream log;
	char cam_name[100];
	char log_name[256];
	char log_dir[256];
	char tmp[512];
	unsigned long start_record_frame;

	void init_cam(int i, bool need_record, const char* _serial_no) {
		idx = i;
		is_need_record = need_record;
		is_open = true;
		is_first_time_init = false;
		is_color = false;
		strncpy(serial_no, _serial_no, 99);
		strncpy(cam_name, _serial_no, 99);

		pid_w_f[idx] = 1;
		pthread_create(&pid_w[idx], NULL, write_thread, &idx);
		//cout << "create write thread " << idx << endl;
	}

	int first_time_init(Json::Value* pConfig, int width, int height, const char* fourcc, float fps) {
		if (is_first_time_init)
			return 0;
		is_first_time_init = 1;
		lost = 0;
		image_width = width;
		image_height = height;

		if (pConfig != NULL) {
			Json::Value t = pConfig->get(serial_no, NULL);
			if (t != NULL) {
				strncpy(cam_name, t.get("name", Json::Value(serial_no)).asCString(), 10);
				Json::Value t_fps = t.get("fps", NULL);
				if (t_fps != NULL)
					fps = t_fps.asFloat();
			}
			else {
				//cout << "warning: config not found---" << serial_no << endl;
				strncpy(cam_name, serial_no, 10);
			}
		}
		open_log();
		begin_store_video(fourcc, fps);

		if (g_show_fps > 0) {
			fc.init_show_timer(1000 / g_show_fps);
		}
		fc.set_tag(serial_no);
		return 1;
	}

	void open_log() {
		time_t t = time(0);
		strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&t));
		sprintf(log_name, "%s_%s", tmp, cam_name);
		create_dir("video"); // create if not exists
		sprintf(log_dir, "video/%s", log_name);
		create_dir(log_dir);
		sprintf(tmp, "video/%s/%s.log", log_name, log_name);
		log.open(tmp, ios::out);
	}
	void close_log() {
		if (log.is_open()) {
			log << "close log " << vw.isOpened() << endl;
			log.flush();
			log.close();
		}
		if (vw.isOpened())
			vw.release();
		if (!g_recorded)
			remove_dir(log_dir);
	}

	bool begin_store_video(const char* fourcc, float fps = 100) {
		log << "camera:\t\t\t" << serial_no << endl;
		log << "size:\t\t\t(" << image_width << "," << image_height << ")" << endl;
		log << "codec:\t\t\t" << fourcc << endl;
		log << "fps:\t\t\t" << fps << endl;
		log << "color:\t\t\t" << is_color << endl;
		char file_name[1000];
		sprintf(file_name, "video/%s/%s.avi", log_name, log_name);
		vw.open(file_name, VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]), fps, Size(image_width, image_height), is_color);
		uc = is_color ? CV_8UC3 : CV_8UC1;
		data_size = image_height * image_width;
		log << "file:\t\t\t" << file_name << endl;
		if (vw.isOpened())
			log << "open:\t\t\t" << get_time_str(tmp) << endl;
		else {
			log << "open video failed:\t\t\t" << get_time_str(tmp) << endl; //need opencv_ffmpeg342_64.dll
			cout << "open video failed: " << file_name << endl;
		}
		init_ts = 0;//get_time();
		start_record_frame = 0;
		return vw.isOpened();
	}
	unsigned long get_start_record_frame() {
		return start_record_frame;
	}
	double get_ts_inc(uint64_t ts) {
		if (init_ts == 0) {
			init_ts = ts;
			start_record_frame = fc.frame_seq;
			log << "start:\t\t\t" << get_time_str(tmp) << " ts:" << get_time() << endl;
		}
		return (ts - init_ts) / 1e9f;
	}

	int process_raw_image(const void* pBuffer, int64_t ts=0) {
		//cout << "ts:" << ts << endl;
		double fps = 0;
		if (fc.update(ts / 1000)) {
			fps = fc.get_fps();
		}
		MSG_INFO* msg;
#ifdef USE_BUFFER 
		msg = new MSG_INFO();
		//cout << "meminfo new " << fc.frame_seq << endl;
		msg->pData = new uint8_t[data_size];
		memcpy(msg->pData, pBuffer, data_size);
#else
		//Mat mat(nImageHeight, nImageWidth, uc, (uint8_t*)pBuffer);
		//msg.pMat = &mat;
		MSG_INFO t_msg;
		t_msg.pData = (uint8_t *)pBuffer;
		msg = &t_msg;
#endif
		msg->pszName = serial_no;
		msg->frame_seq = fc.frame_seq;
		msg->ts = ts; // camera provided timestamp(us)
		msg->pCam = this;
		msg->fps = fps;
#ifdef USE_BUFFER
		if (!put_msg(msg, idx)) {
			cout << "p";
		}
#else
		process_msg(msg, false);
#endif
		return 1;
	}
};

//-------------------------operators-------------------------
class Operator;
struct one_thread_param
{
	int i;
	Operator* p;
};
class Operator {
public:
	int nDeviceNum = 0;
	CAMERA_INFO mCams[CAM_PER_TYPE * 100];
	const char* fourcc;
	bool need_record;
	Json::Value *pConfig;
	int frame_seq[CAM_PER_TYPE * 100];

	int init(bool _need_record, Json::Value* _pConfig, const char* _fourcc) {
		need_record = _need_record;
		pConfig = _pConfig;
		fourcc = _fourcc;
		return 0;
	}

	void init_one_cam(int cam_idx, int camera_type_id, bool need_record, const char* _serial_no, Json::Value* pConfig, int width, int height, const char* fourcc, float fps) {
		if (g_multi_part > 1) {
			char tmp[1000];
			for (int i = 0; i < g_multi_part; i++) {
				sprintf(tmp, "%s_%d", _serial_no, i);
				int idx = cam_idx * g_multi_part + i;
				mCams[idx].init_cam(idx, need_record, tmp);
				mCams[idx].first_time_init(pConfig, width, height, fourcc, fps);
			}
			frame_seq[cam_idx] = 0;
		}
		else {
			mCams[cam_idx].init_cam(cam_idx + camera_type_id * CAM_PER_TYPE, need_record, _serial_no);
			mCams[cam_idx].first_time_init(pConfig, width, height, fourcc, fps);
		}
	}

	void process_one_cam(int cam_idx, const void* pBuffer, int64_t ts = 0){
		if (g_multi_part > 1) {
			mCams[cam_idx * g_multi_part + (frame_seq[cam_idx] % g_multi_part)].process_raw_image(pBuffer, ts);
			frame_seq[cam_idx]++;
			//if (frame_seq[cam_idx] % 1000 == 0) cout << cam_idx << "," << frame_seq[cam_idx] << "," << ts << endl;
		}
		else {
			mCams[cam_idx].process_raw_image(pBuffer, ts);
		}
	}

	virtual int do_open() {
		return 0;
	}
	virtual int do_capture() {
		return 0;
	}
	virtual int stop_capture() {
		return 0;
	}
	virtual void one_frame_func(int i) {}

	static void* opt_thread(void* arg) {
		Operator* opt = (Operator*)arg;
		opt->init(g_need_record, &g_pConfig, g_fourcc);
		opt->do_open();
		opt->do_capture();
		opt->stop_capture();
		return NULL;
	}
	static void* one_thread(void* arg) {
		SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
		one_thread_param* pp = (one_thread_param*)arg;
		//cout << pp->i << endl;
		while (!g_is_stop) {
			pp->p->one_frame_func(pp->i);
		}
		return 0;
	}
	void close_video_and_log() {
		for (int i = 0; i < nDeviceNum; i++) {
			mCams[i].close_log();
		}
	}
	~Operator() {
	}
};

#ifdef USE_HIKVISION
struct MV_CAM_DATA {
	void* handle;
	unsigned char * pData;
	unsigned int nDataSize;
};
class MVOperator:public Operator {
public:
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	int nRet = MV_OK;
	pthread_t pids[CAM_PER_TYPE];
	MV_CAM_DATA cam_data[CAM_PER_TYPE];

	int do_open() {
		nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
		if (MV_OK != nRet) {
			printf("Enum Devices failed! nRet [0x%x]\n", nRet);
			return 0;
		}
		nDeviceNum = stDeviceList.nDeviceNum;
		cout << "[HIKVISION] deviceNum=" << nDeviceNum << endl;
		if (nDeviceNum <= 0) return 0;
		bool is_only_one_camera = (strlen(g_only_one_camera) > 0);
		int _width, _height;
		double _fps;
		for (int i = 0; i < nDeviceNum; i++) {
			MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
			void* handle;
			char* pName = (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
			if (strlen(pName) == 0) pName = (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
			if (is_only_one_camera && (strcmp(g_only_one_camera, pName) != 0)) {
				//nRet = MV_CC_CloseDevice(handle);
				//nRet = MV_CC_DestroyHandle(handle);
				cout << "[HIKVISION] skip " << pName << endl;
				continue;
			}
			nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[i]);
			nRet = MV_CC_OpenDevice(handle);
			if (MV_OK != nRet) {
				//cout << "[HIKVISION] open device failed " << i << endl;
				continue;
			}
			cam_data[i].handle = handle;
			MVCC_INTVALUE stParam;
			MVCC_FLOATVALUE stFloatValue;
			memset(&stParam, 0, sizeof(MVCC_INTVALUE));
			nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
			cam_data[i].nDataSize = stParam.nCurValue;
			cam_data[i].pData = (unsigned char *)malloc(sizeof(unsigned char) * (stParam.nCurValue));

			nRet = MV_CC_GetIntValue(handle, "Width", &stParam);
			_width = stParam.nCurValue;
			nRet = MV_CC_GetIntValue(handle, "Height", &stParam);
			_height = stParam.nCurValue;
			//nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &stEnumValue);
			//enPixelType = MvGvspPixelType(nRet);
			nRet = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &stFloatValue);
			_fps = stFloatValue.fCurValue;

			init_one_cam(i, 3, need_record, pName, pConfig, _width, _height, fourcc, _fps);
			nRet = MV_CC_StartGrabbing(handle);

#ifdef TRIGGER_MODE_OFF
			nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
			if (MV_OK != nRet) {
				printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
			}
#endif
		}
		return 0;
	}
	int do_capture() {
		if (nDeviceNum <= 0) return 0;
		for (int i = 0; i < nDeviceNum; i++) {
			if (mCams[i].is_open) {
				one_thread_param* param = new one_thread_param;
				param->i = i;
				param->p = this;
				pthread_create(&pids[i], NULL, one_thread, param);
			}
		}
		for (int i = 0; i < nDeviceNum; i++) {
			if (mCams[i].is_open) {
				pthread_join(pids[i], NULL);
			}
		}
		return 0;
	}
	void one_frame_func(int i) {
		MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
		MV_CAM_DATA* p = &cam_data[i];
		nRet = MV_CC_GetOneFrameTimeout(p->handle, p->pData, p->nDataSize, &stImageInfo, 5000);
		if (nRet == MV_OK) {
			process_one_cam(i, p->pData, stImageInfo.nHostTimeStamp * 1000000);
		}
	}
	int stop_capture() {
		for (int i = 0; i < nDeviceNum; i++) {
			if (mCams[i].is_open) {
				void* handle = cam_data[i].handle;
				nRet = MV_CC_StopGrabbing(handle);
				nRet = MV_CC_CloseDevice(handle);
				nRet = MV_CC_DestroyHandle(handle);
				delete cam_data[i].pData;
			}
		}
		return 0;
	}
};
MVOperator mv_opt;
#endif

#ifdef USE_DaHeng
class GXOperator:public Operator {
public:
	GX_STATUS emStatus = GX_STATUS_ERROR;
	GX_DEVICE_BASE_INFO m_pBaseinfo[CAM_PER_TYPE];
	GX_DEV_HANDLE hDevs[CAM_PER_TYPE];

	int do_open_one(int i) {
		GX_OPEN_PARAM stOpenParam;
		stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
		stOpenParam.openMode   = GX_OPEN_INDEX;
		
		char      szIndex[10]    = {0};
		int64_t   nValue   = 0;
		bool      bIsColorFilter = false;
		 _itoa(i + 1, szIndex, 10);
		stOpenParam.pszContent = szIndex;

		if (strlen(g_only_one_camera) > 0 && i != 0) {
			mCams[i].is_open = false;
			cout << "[Daheng] skip " << m_pBaseinfo[i].szSN << endl;
			return -1;
		}
		CAMERA_INFO* pCam = &mCams[i];
		emStatus = GXOpenDevice(&stOpenParam, &hDevs[i]);
		GX_VERIFY(emStatus);

		cout << "[Daheng] open " << m_pBaseinfo[i].szSN << endl;
		GX_DEV_HANDLE hDev = hDevs[i];
		emStatus = GXIsImplemented(hDev, GX_ENUM_PIXEL_COLOR_FILTER, &bIsColorFilter);
		pCam->is_color = bIsColorFilter;
		emStatus = GXGetInt(hDev, GX_INT_WIDTH, &nValue);
		int width = nValue;
		emStatus = GXGetInt(hDev, GX_INT_HEIGHT, &nValue);
		int height = nValue;
		double fps = 100;
		emStatus = GXGetFloat(hDev, GX_FLOAT_ACQUISITION_FRAME_RATE, &fps);

		init_one_cam(i, 1, need_record, m_pBaseinfo[i].szSN, pConfig, width, height, fourcc, fps);

		emStatus = GXRegisterCaptureCallback(hDevs[i], &mCams[i], OnFrameCallbackFun);
		return 0;
	}
	static void __stdcall OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame)
	{
		CAMERA_INFO *pCam = (CAMERA_INFO *)(pFrame->pUserParam);
		if (pFrame->status == 0 && !g_is_stop) // TODO: process_one_cam
			pCam->process_raw_image(pFrame->pImgBuf, pFrame->nTimestamp);
	}
	int do_open() {
		emStatus = GXInitLib();
		uint32_t n;
		emStatus = GXUpdateDeviceList(&n, 1000);
		nDeviceNum = n;
		cout << "[Daheng] deviceNum=" << nDeviceNum << endl;
		if (nDeviceNum <= 0) return 0;
		size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
		emStatus = GXGetAllDeviceBaseInfo(m_pBaseinfo, &nSize);
		for (uint32_t i = 0; i < nDeviceNum; i++) {
			do_open_one(i);
		}
		return 0;
	}
	int do_capture() {
		//if (nDeviceNum <= 0) return 0;
		for(uint32_t i = 0; i < nDeviceNum; i++) {
			if (mCams[i].is_open) {
				emStatus = GXSendCommand(hDevs[i], GX_COMMAND_ACQUISITION_START);
			}
		}
		pthread_mutex_lock(&g_stop_mutex);
		cout << "wait start" << endl;
		pthread_cond_wait(&g_stop_cond, &g_stop_mutex);
		cout << "wait end" << endl;
		return 0;
	}
	int stop_capture() {
		cout << "stop_capture" << endl;
		for (int i = 0; i < nDeviceNum; i++)
		{
			GX_DEV_HANDLE hDev = hDevs[i];
			if (mCams[i].is_open)
			{
				emStatus = GXSendCommand(hDev, GX_COMMAND_ACQUISITION_STOP);
				emStatus = GXUnregisterCaptureCallback(hDev);
				emStatus = GXCloseDevice(hDev);
				hDevs[i] = NULL;
			}
		}
		return 0;
	}
};
GXOperator gx_opt;
#endif

#ifdef USE_Basler
class PYOperator:public Operator {
public:
	Pylon::CInstantCameraArray cameras;

	int do_open() {
		try {
			Pylon::PylonInitialize();
			Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
			Pylon::DeviceInfoList_t devices;
			if (tlFactory.EnumerateDevices(devices) == 0) {
				cout << "[Basler] deviceNum=0" << endl;
				return 0;
			}
			nDeviceNum = devices.size();
			cout << "[Basler] deviceNum=" << nDeviceNum << endl;
			if (nDeviceNum == 0) return 0;
			cameras.Initialize(nDeviceNum);
			for (int i = 0; i < nDeviceNum; i++) {
				cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
				cameras[i].Open();
				Pylon::String_t name = cameras[i].GetDeviceInfo().GetModelName();
				GENAPI_NAMESPACE::INodeMap& nodemap = cameras[i].GetNodeMap();
				GENAPI_NAMESPACE::CIntegerPtr width(nodemap.GetNode("Width"));
				GENAPI_NAMESPACE::CIntegerPtr height(nodemap.GetNode("Height"));
				GENAPI_NAMESPACE::CFloatPtr fps(nodemap.GetNode("AcquisitionFrameRate"));
				//GENAPI_NAMESPACE::CFloatPtr et(nodemap.GetNode("ExposureTime"));
				init_one_cam(i, 2, need_record, name.c_str(), pConfig, width->GetValue(), height->GetValue(), fourcc, fps->GetValue());
				cout << "[Basler] open " << name << endl;
			}
			cameras.StartGrabbing();
		}
		catch (const Pylon::GenericException &e) {
			cerr << "[Basler] An exception occurred." << endl << e.GetDescription() << endl;
			return 0;
		}
		return 0;
	}
	int do_capture() {
		if (nDeviceNum <= 0) return 0;
        Pylon::CGrabResultPtr ptrGrabResult;
		while(!g_is_stop && cameras.IsGrabbing()){
			cameras.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
			CAMERA_INFO* pCam = &mCams[cameraContextValue];

			//pCam->first_time_init(pConfig, ptrGrabResult->GetWidth(), ptrGrabResult->GetHeight(), fourcc);
			process_one_cam(cameraContextValue, ptrGrabResult->GetBuffer(), ptrGrabResult->GetTimeStamp());
		}
		return 0;
	}
	int stop_capture() {
		//PylonTerminate();
		return 0;
	}
};
PYOperator py_opt;
#endif

#ifdef USE_FLIR
class SPOperator:public Operator {
public:
	pthread_t pids[CAM_PER_TYPE];
	Spinnaker::SystemPtr system;
	Spinnaker::CameraList camList;

	int do_open() {
		system = Spinnaker::System::GetInstance();
		camList = system->GetCameras();
		nDeviceNum = camList.GetSize();
		cout << "[FLIR] deviceNum=" << nDeviceNum << endl;
		if (nDeviceNum <= 0) return 0;
		try {
			for (int i = 0; i < nDeviceNum; i++) {
				if (g_only_one_camera && i != 0) {
					mCams[i].is_open = false;
					cout << "[FLIR] skip " << endl;
					continue;
				}
				Spinnaker::CameraPtr pPGCam = camList.GetByIndex(i);
				pPGCam->Init();

				Spinnaker::GenICam::gcstring deviceSerialNumber("unknow");
				Spinnaker::GenApi::INodeMap & nodeMap = pPGCam->GetNodeMap();
				Spinnaker::GenApi::INodeMap & nodeMapTLDevice = pPGCam->GetTLDeviceNodeMap();
				Spinnaker::GenApi::CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
				if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
					deviceSerialNumber = ptrStringSerial->GetValue();
				cout << "[FLIR] open " << deviceSerialNumber << endl;
				Spinnaker::GenApi::CIntegerPtr width(nodeMap.GetNode("Width"));
				Spinnaker::GenApi::CIntegerPtr height(nodeMap.GetNode("Height"));
				Spinnaker::GenApi::CFloatPtr fps = nodeMap.GetNode("AcquisitionFrameRate");
				Spinnaker::GenApi::CFloatPtr et = nodeMap.GetNode("ExposureTime");
				init_one_cam(i, 0, need_record, deviceSerialNumber.c_str(), pConfig, width->GetValue(), height->GetValue(), fourcc, 1e6 / et->GetValue());
				pPGCam->BeginAcquisition();
			}
		}
		catch (Spinnaker::Exception &e) {
			cerr << "[FLIR] An exception occurred." << endl << e.what() << endl;
		}
		return 0;
	}
	int do_capture() {
		if (nDeviceNum <= 0) return 0;
		for (int i = 0; i < nDeviceNum; i++) {
			if (mCams[i].is_open) {
				one_thread_param* param = new one_thread_param;
				param->i = i;
				param->p = this;
				pthread_create(&pids[i], NULL, one_thread, param);
			}
		}
		for (int i = 0; i < nDeviceNum; i++) {
			if (mCams[i].is_open) {
				pthread_join(pids[i], NULL);
			}
		}
		return 0;
	}
	void one_frame_func(int i) {
		Spinnaker::CameraPtr pPGCam = camList.GetByIndex(i);
		Spinnaker::ImagePtr pResultImage = pPGCam->GetNextImage();
		if (pResultImage->IsIncomplete()) {
			cout << "[FLIR] Image incomplete: " << Spinnaker::Image::GetImageStatusDescription(pResultImage->GetImageStatus()) << endl;
			return;
		}
		//pCam->first_time_init(pConfig, pResultImage->GetWidth(), pResultImage->GetHeight(), fourcc, pCam->fFps);
		process_one_cam(i, pResultImage->GetData(), pResultImage->GetTimeStamp());
		pResultImage->Release();
	}
	int stop_capture() {
		for (int i = 0; i < nDeviceNum; i++) {
			if (mCams[i].is_open) {
				Spinnaker::CameraPtr pPGCam = camList.GetByIndex(i);
				pPGCam->EndAcquisition();
				pPGCam->DeInit();
			}
		}
		camList.Clear();
		system->ReleaseInstance();
		return 0;
	}
};
SPOperator sp_opt;
#endif

//--------------------------write thread-----------------------------
void process_msg(MSG_INFO* msg, bool need_delete) {
	/*Mat img = imread("t.jpg", 1);
	imshow(msg->pCam->pszName, img);
	waitKey(1);
	return;*/
	CAMERA_INFO* pCam = msg->pCam;
	Mat m(pCam->image_height, pCam->image_width, pCam->uc, msg->pData);
	if (g_need_record && pCam->vw.isOpened()) {
		pCam->vw.write(m);
		sprintf(pCam->tmp, "%d %f\n", msg->frame_seq, pCam->get_ts_inc(msg->ts));
		pCam->log << pCam->tmp;
		//pCam->log.flush(); //for test
	}
	if (g_show_fps >= 0 && pCam->fc.update_show_timer()) {
		if (g_show_fps == 0) {
			destroyAllWindows();
			g_show_fps = -1;
		}
		else {
			imshow(pCam->serial_no, m);
			//resizeWindow(pCam->pszName, pCam->nImageWidth / 2, pCam->nImageHeight / 2);
			waitKey(1);
		}
	}
	if (msg->fps > 0) {
		sprintf(pCam->tmp, "[%s]\t%.2f\t%d\n", msg->pszName, msg->fps, msg->frame_seq - pCam->get_start_record_frame());
		if (g_output) cout << pCam->tmp;
	}
	if (need_delete) {
		//cout << "meminfo delete2 " << msg->frame_seq << endl;
		del_msg(msg);
	}
}

void* write_thread(void *arg) {
	int idx = *((int *)arg);
	//SetThreadAffinityMask(GetCurrentThread(), 1<<idx);
	setNumThreads(1);
	cout << "write thread on " << idx << endl;
	CAMERA_INFO* pCam = NULL;
	while (!g_is_stop) {
		MSG_INFO* msg = g_msg_queue[idx].take();
		if (msg) {
			pCam = msg->pCam;
			process_msg(msg, true);
		}
	}

	pthread_mutex_lock(&g_mutex);
	cout << "write_thread end " << idx << endl;// << " end " << pCam->cam_name << endl;
	pthread_mutex_unlock(&g_mutex);
	if (pCam) pCam->close_log();
	return NULL;
}

void write_thread_interrupt() {
	//cout << "write_thread_interrupt" << endl;
	for (int p = 0; p < MSG_QUEUE_NUM; p++) {
		if (pid_w_f[p] != 0) {
			g_msg_queue[p].interrupt();
			pthread_join(pid_w[p], NULL);
			cout << "write_thread_interrupt ok " << p << endl;
		}
	}
}

//--------------------------main-----------------------------

void close_all() {
#ifdef USE_DaHeng
	gx_opt.close_video_and_log();
#endif
#ifdef USE_Basler
	py_opt.close_video_and_log();
#endif
#ifdef USE_FLIR
	sp_opt.close_video_and_log();
#endif
#ifdef USE_HIKVISION
	mv_opt.close_video_and_log();
#endif
}

void init_config()
{
	Json::Value config = read_config("config.json");
	g_need_record = config.get("need_record", Json::Value(0)).asBool();
	if (g_need_record)
		g_recorded = true;
	g_show_fps = config.get("show_fps", Json::Value(30)).asInt();
	g_pConfig = config.get("cameras", Json::nullValue);
	g_buffer_frames = config.get("buffer", Json::Value(100)).asInt();
	strncpy(g_fourcc, config.get("fourcc", Json::Value("DIVX")).asCString(), 4);
	strncpy(g_only_one_camera, config.get("one_camera", Json::Value("")).asCString(), 100);
	g_multi_part = config.get("multi_part", Json::Value(g_multi_part)).asInt();
	g_fourcc[4] = 0;
}

void* camera_thread(void *) {
	pthread_t pid_gx, pid_py, pid_sp, pid_mv;
#ifdef USE_DaHeng
	pthread_create(&pid_gx, NULL, gx_opt.opt_thread, &gx_opt);
#endif
#ifdef USE_Basler
	pthread_create(&pid_py, NULL, py_opt.opt_thread, &py_opt);
#endif
#ifdef USE_FLIR
	pthread_create(&pid_sp, NULL, sp_opt.opt_thread, &sp_opt);
#endif
#ifdef USE_HIKVISION
	pthread_create(&pid_mv, NULL, mv_opt.opt_thread, &mv_opt);
#endif

#ifdef USE_DaHeng
	pthread_join(pid_gx, NULL);
#endif
#ifdef USE_Basler
	pthread_join(pid_py, NULL);
#endif
#ifdef USE_FLIR
	pthread_join(pid_sp, NULL);
#endif
#ifdef USE_HIKVISION
	pthread_join(pid_mv, NULL);
#endif
	//g_log << "all end:\t\t\t" << get_time_str(g_tmp) << endl;
	//close_all();
	return NULL;
}

void CreateDumpFile(LPCSTR lpstrDumpFilePathName, EXCEPTION_POINTERS *pException)
{
	HANDLE hDumpFile = CreateFile(lpstrDumpFilePathName, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
	dumpInfo.ExceptionPointers = pException;
	dumpInfo.ThreadId = GetCurrentThreadId();
	dumpInfo.ClientPointers = TRUE;
	MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hDumpFile, MiniDumpNormal, &dumpInfo, NULL, NULL);
	CloseHandle(hDumpFile);
}
LONG ApplicationCrashHandler(EXCEPTION_POINTERS *pException)
{
	CreateDumpFile("CRASH.dmp", pException);
	close_all();
	return EXCEPTION_EXECUTE_HANDLER;
}

void wait_threads_and_close_all()
{
	g_is_stop = true;
	pthread_cond_signal(&g_stop_cond);
	pthread_join(pid_c, NULL);
	write_thread_interrupt();
	close_all();
}

BOOL CtrlHandler(DWORD fdwCtrlType)
{
	if (fdwCtrlType == CTRL_C_EVENT || fdwCtrlType == CTRL_CLOSE_EVENT) {
		wait_threads_and_close_all();
		return true;
	}
	return false;
}


int main(int argc, char** argv) {
	//_CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_LEAK_CHECK_DF);
	SetUnhandledExceptionFilter((LPTOP_LEVEL_EXCEPTION_FILTER)ApplicationCrashHandler);
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE);
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    init_time();
	init_config();
	if (argc > 1) {
		if (argv[1][0] == '0') {
			g_need_record = true;
			g_show_fps = 0;
			g_recorded = true;
		}
	}

	pthread_mutex_init(&g_mutex, NULL);
	pthread_mutex_init(&g_stop_mutex, NULL);
	pthread_cond_init(&g_stop_cond, NULL);
	pthread_create(&pid_c, NULL, camera_thread, NULL);

	while (1)
	{
		g_cmd = getchar();
		cout << "cmd: " << g_cmd << endl;
		if(g_cmd == 'q') {
			break;
		} else if(g_cmd == 'c') {
			g_show_fps = 0;
			g_need_record = true;
			g_recorded = true;
			cout << "start record" << endl;
		} else if (g_cmd == 's') {
			g_show_fps = 30;
		} else if (g_cmd == 'r') {
			g_show_fps = 30;
			g_need_record = false;
			cout << "stop record" << endl;
		} else if (g_cmd == 'x') {
			g_output = false;
		}
	}
	wait_threads_and_close_all();
	cout << endl << "press any key to exit ..." << endl;
	getchar();
	return 0;
}
