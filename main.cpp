/**
 * ./ipconvif -cIp '192.168.1.53' -cUsr 'admin' -cPwd 'admin' -fIp '192.168.1.51:21' -fUsr 'ftpuser' -fPwd 'Ftpftp123
 *
 */

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "stdio.h"
#include "wsdd.nsmap"
#include "plugin/wsseapi.h"
#include "plugin/wsaapi.h"
#include <openssl/rsa.h>
#include "ErrorLog.h"
#include "CMT.h"
 
#include "include/soapDeviceBindingProxy.h"
#include "include/soapMediaBindingProxy.h"
#include "include/soapPTZBindingProxy.h"

#include "include/soapPullPointSubscriptionBindingProxy.h"
#include "include/soapRemoteDiscoveryBindingProxy.h" 

#include <ncurses.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <stdarg.h>  // For va_start, etc.
#include <memory>    // For std::unique_ptr
#include <ctime>
#include <sstream>	// For stringstream
//#include "Snapshot.hpp"
#include <algorithm>
#include <cstring>
#include <fstream>

#define PI 3.14159265
#define MAX_HOSTNAME_LEN 128
#define MAX_LOGMSG_LEN 256 

using cmt::CMT;

class Timer
{
public:
    Timer() { clock_gettime(CLOCK_REALTIME, &beg_); }

    double elapsed() {
        clock_gettime(CLOCK_REALTIME, &end_);
        return end_.tv_sec - beg_.tv_sec +
            (end_.tv_nsec - beg_.tv_nsec) / 1000000000.;
    }

    void reset() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
    timespec beg_, end_;
};

struct soap *soap = soap_new();

cv::String face_cascade_name = "haarcascade_frontalface_alt.xml";
cv::String profileface_cascade_name = "haarcascade_profileface.xml";
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier profile_cascade;
PTZBindingProxy proxyPTZ;
std::vector<cv::Rect> detected_faces;
//cv::Point detected_face(0.0,0.0);
//cv::Point moving_face(0.0, 0.0);
bool tracking = false;
bool moving = false;
bool camera_control = true;
bool last = false;
cv::Rect rect;
cv::VideoCapture capture;
float x = 0, y = 0, z = 0;
float border_x = 320;
float border_y = 240;
char *fifo = "ipcfifo";
int fd;
char buf[1024];

float width, height;

float zoom_factor = 0.1;

float speed_x = 0.2;
float speed_y = 0.3;

char szHostName[MAX_HOSTNAME_LEN] = {0};
char szPTZName[MAX_HOSTNAME_LEN] = {0};
char szStreamName[MAX_HOSTNAME_LEN] = {0};

char *camIp;
char *camUsr;
char *camPwd;

cv::Mat im;

int control(void)
{
    int ch = getch();
 
    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}

void *CaptureImages(void *threadid)
{
	while(1)
	{
		while(!capture.isOpened())
		{
			printw("Error opening video stream\n");
            		refresh();
			sleep(3);
			capture.open(szStreamName);
		}
		width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
		height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
		while (capture.isOpened())
		{
			capture >> im;
			if (im.empty())
			{
				break;
			}
		}
	}
	im.release();
	pthread_exit(NULL);
	return NULL;
}

void StopMove ()
{
	bool success = false;
	while(!success)
	{
		_tptz__Stop *tptz__Stop = soap_new__tptz__Stop(soap, -1);
		_tptz__StopResponse *tptz__StopResponse = soap_new__tptz__StopResponse(soap, -1);
						
		tptz__Stop->ProfileToken = "Profile_1";
		if(SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
    		{		
  			printw("TOKEN ERROR\n");
            		refresh();
     		}	
       		if(SOAP_OK == proxyPTZ.Stop(tptz__Stop, tptz__StopResponse))
		{
			printw("STOPPED2\n");
            		refresh();
			success = true;
		}
	}
}

void *ContMove(void *threadid)
{
	while(1)
	{
		if (moving)
		{
			if(last)
			{
				moving = false;
				last = false;
				x = 0.0;
				y = 0.0;
				z = 0.0;
			}
			_tptz__ContinuousMove *tptz__ContinuousMove = soap_new__tptz__ContinuousMove(soap, -1);
			_tptz__ContinuousMoveResponse *tptz__ContinuousMoveResponse = soap_new__tptz__ContinuousMoveResponse(soap, -1);

			tt__PTZSpeed *Speed = soap_new_tt__PTZSpeed(soap, -1);
			
			Speed->PanTilt = new tt__Vector2D;
			Speed->Zoom = new tt__Vector1D;
			Speed->PanTilt->x = x;
			Speed->PanTilt->y = y;
			Speed->Zoom->x = z;
			
			tptz__ContinuousMove->ProfileToken = "Profile_1";
			tptz__ContinuousMove->Velocity = Speed;
			
			if (SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
			{
				printw("TOKEN ERROR\n");
            			refresh();
			}
			//soap_wsse_add_Timestamp(proxyPTZ.soap, "Time", 10);
			if (SOAP_OK == proxyPTZ.ContinuousMove(tptz__ContinuousMove, tptz__ContinuousMoveResponse))
			{
				printw("MOVED X: %f\n", tptz__ContinuousMove->Velocity->PanTilt->x);
            			refresh();
				printw("MOVED Y: %f\n", tptz__ContinuousMove->Velocity->PanTilt->y);
            			refresh();
			}
			else
			{
				printw("ERROR\n");
				x = 0.0;
				y = 0.0;
				z = 0.0;
				moving = false;
				StopMove();
				/*_tptz__Stop *tptz__Stop = soap_new__tptz__Stop(soap, -1);
				_tptz__StopResponse *tptz__StopResponse = soap_new__tptz__StopResponse(soap, -1);
						
				tptz__Stop->ProfileToken = "Profile_1";
				if(SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
        			{		
  					printw("TOKEN ERROR\n");
            				refresh();
        			}
				soap_wsse_add_Timestamp(proxyPTZ.soap, "Time", 10);
       				if(SOAP_OK == proxyPTZ.Stop(tptz__Stop, tptz__StopResponse))
				{
					printw("STOPPED2\n");
            				refresh();
				}	*/	
			}
				
			soap_destroy(soap); 
    			soap_end(soap);
			
			usleep(200000);
		}
		//usleep(200000);
	}
	pthread_exit(NULL);
	return NULL;
}

void WriteToStat ()
{
	std::ofstream myfile;
	myfile.open("stat", std::ios::out);
	myfile << camera_control << std::endl;
	myfile << tracking << std::endl;
	if (border_x == width/2)
	{
		myfile << "1" << std::endl;
	}
	else if (border_x == width/3)
	{
		myfile << "0" << std::endl;
	}
	else if (border_x == (width*2)/3)
	{
		myfile << "2" << std::endl;
	}
	myfile.close();	
}
	
char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

void PrintErr(struct soap* _psoap)
{
	fflush(stdout);
	processEventLog(__FILE__, __LINE__, stdout, "error:%d faultstring:%s faultcode:%s faultsubcode:%s faultdetail:%s", _psoap->error, 
	*soap_faultstring(_psoap), *soap_faultcode(_psoap),*soap_faultsubcode(_psoap), *soap_faultdetail(_psoap));
}

void detect(cv::Mat frame);
void track(cv::Mat frame0);
void move(cv::Point point);

int main(int argc, char* argv[])
{
	//cv::Mat frame;
	
	initscr();

    	cbreak();
    	noecho();
    	nodelay(stdscr, TRUE);

    	scrollok(stdscr, TRUE);
	
	if(!face_cascade.load(face_cascade_name)){printw("Error opening face cascade\n");
            		refresh();endwin(); return -1;}
	//if(!profile_cascade.load(profileface_cascade_name)){std::cout <<"Error loading profile cascade!"<<std::endl; return -1;}
	
	// Proxy declarations
	//PTZBindingProxy proxyPTZ;
	DeviceBindingProxy proxyDevice;

	if(!(  cmdOptionExists(argv, argv+argc, "-cIp")
			&& cmdOptionExists(argv, argv+argc, "-cUsr")
			&& cmdOptionExists(argv, argv+argc, "-cPwd") ))
   	{
    		//std::cout  <<  "usage: ./ipconvif -cIp [<camera-ip>:<port>] -cUsr <cam-id> -cPwd <cam-pwd>\n";
		printw("usage: ./ipconvif -cIp [<camera-ip>:<port>] -cUsr <cam-id> -cPwd <cam-pwd>\n");
            	refresh();
		endwin();
    		return -1;
    	}

    	camIp = getCmdOption(argv, argv+argc, "-cIp");
   	camUsr = getCmdOption(argv, argv+argc, "-cUsr");
    	camPwd = getCmdOption(argv, argv+argc, "-cPwd");

    	if (!(camIp && camUsr && camPwd ))
    	{
    		processEventLog(__FILE__, __LINE__, stdout, "Error: Invalid args (All args are required!)");
		endwin();
    		return -1;
    	}

    	strcat(szHostName, "http://");
	strcat(szHostName, camIp);
	strcat(szHostName, ":80/onvif/device_service");

	strcat(szPTZName, "http://");
	strcat(szPTZName, camIp);
	strcat(szPTZName, ":80/onvif/PTZ");

	strcat(szStreamName, "rtsp://");
	strcat(szStreamName, camIp);
	strcat(szStreamName, ":554//Streaming/Channels/2");

	proxyDevice.soap_endpoint = szHostName;
	proxyPTZ.soap_endpoint = szPTZName;

	// Register plugins
	soap_register_plugin(proxyDevice.soap, soap_wsse);
	soap_register_plugin(proxyPTZ.soap, soap_wsse);

	soap_wsse_add_Timestamp(proxyPTZ.soap, "Time", 0);

	//struct soap *soap = soap_new();
	// For DeviceBindingProxy
	if (SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyDevice.soap, NULL, camUsr, camPwd))
	{
		PrintErr(proxyDevice.soap);
		refresh();
		//endwin();
		//return -1;
	}

	if (SOAP_OK != soap_wsse_add_Timestamp(proxyDevice.soap, "Time", 10)) 
	{
		PrintErr(proxyDevice.soap);
		refresh();
		//endwin();
		//return -1;
	}
    
    	// Get Device info
    	_tds__GetDeviceInformation *tds__GetDeviceInformation = soap_new__tds__GetDeviceInformation(soap, -1);
    	_tds__GetDeviceInformationResponse *tds__GetDeviceInformationResponse = soap_new__tds__GetDeviceInformationResponse(soap, -1);
    
    	if (SOAP_OK == proxyDevice.GetDeviceInformation(tds__GetDeviceInformation, tds__GetDeviceInformationResponse))
    	{
        	processEventLog(__FILE__, __LINE__, stdout, "-------------------DeviceInformation-------------------");
        	processEventLog(__FILE__, __LINE__, stdout, "Manufacturer:%sModel:%s\r\nFirmwareVersion:%s\r\nSerialNumber:%s\r\nHardwareId:%s", tds__GetDeviceInformationResponse->Manufacturer.c_str(),
                        tds__GetDeviceInformationResponse->Model.c_str(), tds__GetDeviceInformationResponse->FirmwareVersion.c_str(),
                        tds__GetDeviceInformationResponse->SerialNumber.c_str(), tds__GetDeviceInformationResponse->HardwareId.c_str());
		refresh();
    	}
    	else
    	{
        	PrintErr(proxyDevice.soap);
		refresh();
		//endwin();
        	//std::cout << "Error getting device information"<<std::endl;
		//return -1;
    	}
    
    	// DeviceBindingProxy ends
    	soap_destroy(soap); 
    	soap_end(soap);
	
	capture.open(szStreamName);
	capture.set(cv::CAP_PROP_BUFFERSIZE, 3);
	
	while(!capture.isOpened()){printw("Error opening video stream\n");
            		refresh();
			sleep(3);
			capture.open(szStreamName);/*endwin(); return -1;*/}
	//capture.grab();
	//capture.retrieve(im);
	WriteToStat();
	
	width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
	height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
	
	pthread_t capture_thread, move_thread;
	int thread_id1 = 0, thread_id2 = 1;
	pthread_create(&capture_thread, NULL,
		CaptureImages, (void *)thread_id1);
	pthread_create(&move_thread, NULL,
		ContMove, (void *)thread_id2);
	/*if (rc)
	{
		endwin();
		close(fd);
	}*/
	
	fd = open(fifo, O_RDONLY | O_NONBLOCK);
	while(1)
	{	
		if(!im.empty())
		{
			if( tracking == false)
			{
				detect(im);
			}
			else
			{
				track(im);
			}
		}
		else
		{
			printw("No captured frame!\n");
			refresh();
			//capture.open(szStreamName);
		}
		if (control())
		{
           	 	int ch = getch();
			if(ch == 113)
			{
				//x = 0.0;
				//y = 0.0;
				last = true;
				usleep(300000);
				printw("Exit\n");
            			refresh();
				endwin();
				close(fd);
				return 0;
			}
			else if(ch == 116)
			{
				printw("Tracking at specified spot\n");
            			refresh();
				tracking = true;
				rect.x = 200;
				rect.y = 150;
				rect.height = 50;
				rect.width = 50;
				WriteToStat();
			}
			else if(ch == 112)
			{
				printw("Moving to a specific point\n");
            			refresh();
				cv::Point face(0.0, 0.0);
				move(face);
			}	
		}
		if(read(fd, &buf, sizeof(buf)))
		{
			if(*buf == 107)
			{
				//x = 0.0;
				//y = 0.0;
				last = true;
				usleep(300000);
				printw("Exit\n");
            			refresh();
				endwin();
				return 0;
			}
			else if(*buf == 116)
			{
				printw("Tracking at specified spot\n");
            			refresh();
				tracking = true;
				rect.x = 200;
				rect.y = 250;
				rect.height = 50;
				rect.width = 50;
				WriteToStat();
			}
			else if(*buf == 112)
			{
				printw("Moving to a specific point\n");
            			refresh();
				cv::Point face(0.0, 0.0);
				move(face);
			}
			else if(*buf == 99)
			{
				printw("Camera control enabled\n");
            			refresh();
				camera_control = true;
				WriteToStat();
			}
			else if(*buf == 102)
			{
				//x = 0.0;
				//y = 0.0;
				last = true;
				usleep(300000);
				printw("Camera control disabled\n");
            			refresh();
				camera_control = false;
				WriteToStat();
			}
			else if(*buf == 43)
			{
				printw("Speed increased\n");
            			refresh();
				if(speed_y <= 0.6)
				{
					speed_x = speed_x + 0.1;
					speed_y = speed_y + 0.1;
				}
			}
			else if(*buf == 45)
			{
				printw("Speed decreased\n");
            			refresh();
				if(speed_y > 0.1)
				{
					speed_x = speed_x - 0.1;
					speed_y = speed_y - 0.1;
				}
			}
			else if(*buf == 109)
			{
				printw("Center position\n");
            			refresh();
				//detected_face.x == 0;
				border_x = width/2;
				border_y = height/2;
				WriteToStat();
			}
			else if(*buf == 108)
			{
				printw("Left position\n");
            			refresh();
				border_x = width/3;
				border_y = height/3;
				//detected_face.x == 0;
				WriteToStat();
			}
			else if(*buf == 114)
			{
				printw("Right position\n");
            			refresh();
				border_x = (width*2)/3;
				border_y = height/3;
				//detected_face.x == 0;
				WriteToStat();
			}
			else if(*buf == 110)
			{
				printw("new camera\n");
				refresh();
				char temp[128];
				int i = 0, k = 0, j;
				char c;
				while(1)
				{
					j = 0;
					if(buf[i] == ' ')
					{
						i++;
						while(buf[i] != ' ' || buf[i] != '\n')
						{
							temp[j] = buf[i];
							j++;
							i++;
						}
						temp[j] = '\0';
						if(k == 0)
						{
							strcpy(camIp, temp);
							k++;
						}
						else if(k == 1)
						{
							strcpy(camUsr, temp);
							k++;
						}
						else if (k == 2)
						{
							strcpy(camPwd, temp);
							k++;
						}
						else 
						{
							break;
						}
					}
				}
				printw("new address: %S\n", camIp);
				strcpy(szHostName, "http://");
				strcat(szHostName, camIp);
				strcat(szHostName, ":80/onvif/device_service");

				strcpy(szPTZName, "http://");
				strcat(szPTZName, camIp);
				strcat(szPTZName, ":80/onvif/PTZ");

				strcpy(szStreamName, "rtsp://");
				strcat(szStreamName, camIp);
				strcat(szStreamName, ":554//Streaming/Channels/2");	
			}
					
		}
		//cv::waitKey(100);

	}
	close(fd);
	endwin();
	return 0;
}

void detect(cv::Mat frame)
{
	//std::ofstream myfile;
	//myfile.open("table_haar_miem.csv", std::ios::app);
	//Timer tmr2;
	//tmr2.reset();
	std::vector<cv::Rect> faces;
	cv::Mat frame_gray;
	cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
	cv::equalizeHist(frame_gray,frame_gray);
	int i, dif;
	
	face_cascade.detectMultiScale(frame_gray, faces, 1.2, 5, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30,30));
	if(faces.size()!= 0)
	{
		/*profile_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30,30));
		if(faces.size()!=0)
		{
			//std::cout<<"Found " << faces.size() << " profile faces" << std::endl;
			cv::Point profile(faces[0].x*2.72 + faces[0].width*1.36, faces[0].y*1.875 + faces[0].height*0.9375);
			move(profile);	
		}	
		else
		{
			//std::cout<<"Found no faces" << std::endl;
		}*/
		if(detected_faces.size() == 0){detected_faces.push_back(faces[0]);}
		else
		{
			i = detected_faces.size()-1;
			dif = faces[0].x - detected_faces[i].x;
			if(abs(dif)>20)
			{
				for(i;i>=0;i--){detected_faces.pop_back();}
			}
			else
			{
				if(i == 8)
				{
					printw("Found a face, tracking\n");
            				refresh();
					//std::cout<<"Found a face"<<std::endl;
					//cv::Point face(faces[0].x*2.72 + faces[0].width*1.36, faces[0].y*1.875 + faces[0].height*0.9375);
					//detected_face = face;		
					tracking = true;
					rect = faces[0];
					rect.height = rect.height - 5;
					rect.width = rect.width - 10;
					rect.x = rect.x + 10;
					rect.y = rect.y + 10;
					WriteToStat();
					for(i;i>=0;i--){detected_faces.pop_back();}
				}
				else
				{
					detected_faces.push_back(faces[0]);
				}
			}
		}
	}
	//double t = tmr2.elapsed();
	//std::cout << t << std::endl;
	//myfile << t << std::endl;
	//myfile.close();
}

void track(cv::Mat frame0)
{
	int verbose_flag = 0;
	FILELog::ReportingLevel() = verbose_flag ? logDEBUG : logINFO;
	Output2FILE::Stream() = stdout; //Log to stdout
	//std::ofstream myfile;
	//myfile.open("table_tracking_auditory.csv", std::ios::app);
	//Timer tmr2;
	//tmr2.reset();
	cv::Mat frame0_gray;
	CMT cmt;

	if (frame0.channels() > 1) {
		cvtColor(frame0, frame0_gray, CV_BGR2GRAY);
		equalizeHist(frame0_gray, frame0_gray);
	}
	else {
		frame0_gray = frame0;
	}
	cmt.initialize(frame0_gray, rect);
	//double t = tmr2.elapsed();
	//myfile << t << std::endl;
	while (tracking)
	{
		//tmr2.reset();
		//capture.grab();
		//capture.retrieve(frame);
		//if (frame.empty()) break;

		cv::Mat frame_gray;

		if (im.channels() > 1) {
			cvtColor(im, frame_gray, CV_BGR2GRAY);
			equalizeHist(frame_gray, frame_gray);
		}
		else {
			frame_gray = im;
		}
		cmt.processFrame(frame_gray);
		//t = tmr2.elapsed();
		//myfile << t << std::endl;
		if(cmt.points_active.size()<5)
		{
			printw("Stopped tracking1\n");
            		refresh();
			//x = 0.0;
			//y = 0.0;
			last = true;
			//usleep(300000);
			tracking = false;
			WriteToStat();
			break;
		}

		//char key = display(frame, cmt);
		if (control())
		{
           	 	int ch = getch();
			if(ch == 115)
			{
				printw("Stopped tracking2\n");
            			refresh();
				//x = 0.0;
				//y = 0.0;
				last = true;
				//usleep(300000);
				tracking = false;
				//moving = false;
				//StopMove();
				//myfile.close();
				//detected_face.x = 0;
				//break;
				WriteToStat();
				break;
			}
			else if(ch == 99)
			{
				printw("Camera control enabled\n");
            			refresh();
				camera_control = true;
				x = 0.0;
				y = 0.0;
				WriteToStat();
			}
			else if(ch == 102)
			{
				printw("Camera control disabled\n");
            			refresh();
				camera_control = false;
				//x = 0.0;
				//y = 0.0;
				last = true;
				usleep(300000);
				WriteToStat();
			}
			else if(ch == 43)
			{
				printw("Speed increased\n");
            			refresh();
				speed_x = speed_x + 0.1;
				speed_y = speed_y + 0.1;
			}
			else if(ch == 45)
			{
				printw("Speed decreased\n");
            			refresh();
				speed_x = speed_x - 0.1;
				speed_y = speed_y - 0.1;
			}
			else if(ch == 109)
			{
				printw("Center position\n");
            			refresh();
				//detected_face.x == 0;
				border_x = width/2;
				border_y = height/2;
				WriteToStat();
				printw("border X: %f\n", border_x);
            			refresh();
				printw("border Y: %f\n", border_y);
				refresh();
			}
			else if(ch == 108)
			{
				printw("Left position\n");
            			refresh();
				border_x = width/3;
				border_y = height/3;
				WriteToStat();
				printw("border X: %f\n", border_x);
            			refresh();
				printw("border Y: %f\n", border_y);
				refresh();
				//detected_face.x == 0;
			}
			else if(ch == 114)
			{
				printw("Right position\n");
            			refresh();
				border_x = (width*2)/3;
				border_y = height/3;
				WriteToStat();
				printw("border X: %f\n", border_x);
            			refresh();
				printw("border Y: %f\n", border_y);
				refresh();
				//detected_face.x == 0;
			}
		}
		if(read(fd, &buf, sizeof(buf)))
		{
			if(*buf == 115)
			{
				printw("Stopped tracking\n");
            			refresh();
				tracking = false;
				//x = 0.0;
				//y = 0.0;
				last = true;
				//usleep(300000);
				//StopMove();
				WriteToStat();
				break;
				//myfile.close();
			}
			else if(*buf == 99)
			{
				printw("Camera control enabled\n");
            			refresh();
				camera_control = true;
				x = 0.0;
				y = 0.0;
				WriteToStat();
			}
			else if(*buf == 102)
			{
				printw("Camera control disabled\n");
            			refresh();
				camera_control = false;
				//x = 0.0;
				//y = 0.0;
				last = true;
				usleep(300000);
				//StopMove();
				WriteToStat();
			}
			else if(*buf == 43)
			{
				printw("Speed increased\n");
            			refresh();
				if(speed_y <= 0.6)
				{
					speed_x = speed_x + 0.1;
					speed_y = speed_y + 0.1;
				}
			}
			else if(*buf == 45)
			{
				printw("Speed decreased\n");
            			refresh();
				if(speed_y > 0.1)
				{
					speed_x = speed_x - 0.1;
					speed_y = speed_y - 0.1;
				}
			}
			else if(*buf == 109)
			{
				printw("Center position\n");
            			refresh();
				//detected_face.x == 0;
				border_x = width/2;
				border_y = height/2;
				WriteToStat();
			}
			else if(*buf == 108)
			{
				printw("Left position\n");
            			refresh();
				border_x = width/3;
				border_y = height/3;
				WriteToStat();
				//detected_face.x == 0;
			}
			else if(*buf == 114)
			{
				printw("Right position\n");
            			refresh();
				border_x = (width*2)/3;
				border_y = height/3;
				WriteToStat();
				//detected_face.x == 0;
			}
		}
		if ((abs(cmt.bb_rot.size.height - rect.height) > 50 || abs(cmt.bb_rot.size.width - rect.width) > 50))
		{
			printw("Stopped tracking3\n");
            		refresh();
			tracking = false;
			//x = 0.0;
			//y = 0.0;
			last = true;
			WriteToStat();
			//usleep(300000);
			break;
			//StopMove();
		}
		else if(camera_control && tracking)
		{
			if(abs(cmt.bb_rot.center.x - border_x) > 35 || abs(cmt.bb_rot.center.y - border_y) > 30)
			{
				if((cmt.bb_rot.center.x - border_x)/1000 > 0.035)
				{
					x = (cmt.bb_rot.center.x - border_x)/1000 + speed_x;
					if(width - cmt.bb_rot.center.x < 90)
					{	
						y = y + 0.4; 	
					}
				}
				else if ((cmt.bb_rot.center.x - border_x)/1000 < -0.035)
				{
					x = (cmt.bb_rot.center.x - border_x)/1000 - speed_x;
					if(width - cmt.bb_rot.center.x > (width - 90))
					{	
						y = y - 0.4; 	
					}
				}
				else
				{
					x = 0;
				}
				if((cmt.bb_rot.center.y - border_y)/1000 > 0.03)
				{
					y = -((cmt.bb_rot.center.y - border_y)/1000 + speed_y);
					if(height - cmt.bb_rot.center.y < 90)
					{	
						y = y - 0.4; 	
					}
				}
				else if ((cmt.bb_rot.center.y - border_y)/1000 < -0.03)
				{
					y = -((cmt.bb_rot.center.y - border_y)/1000 - speed_y);
					if(height - cmt.bb_rot.center.y > (height - 90))
					{	
						y = y + 0.4; 	
					}
				}
				else
				{
					y = 0;
				}
				
				/*/if((width*zoom_factor) - cmt.bb_rot.size.height > 10)
				{
					z = 0.01;
					rect.height = rect.height + 10;
				}
				else if ((width*zoom_factor) - cmt.bb_rot.size.height < 10)
				{
					z = -0.01;
					rect.height = rect.height - 10;
				}
				else
				{
					z = 0.0;	
				}*/
				moving = true;
			}
			else if (moving)
			{
				last = true;
				usleep(300000);
				//StopMove();
				/*_tptz__Stop *tptz__Stop = soap_new__tptz__Stop(soap, -1);
				_tptz__StopResponse *tptz__StopResponse = soap_new__tptz__StopResponse(soap, -1);
						
				tptz__Stop->ProfileToken = "Profile_1";
				if(SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
        			{		
  					printw("TOKEN ERROR\n");
            				refresh();
        			}
				soap_wsse_add_Timestamp(proxyPTZ.soap, "Time", 10);
       				if(SOAP_OK == proxyPTZ.Stop(tptz__Stop, tptz__StopResponse))
				{
					printw("STOPPED\n");
            				refresh();
				}		
				soap_destroy(soap);
				soap_end(soap);*/
			}
				/*if(detected_face.x == 0)
				{		
					detected_face.x = cmt.bb_rot.center.x;
					detected_face.y = cmt.bb_rot.center.y;
					cv::Point face(detected_face.x * 3, detected_face.y * 2.25);
					printw("detected face x == 0\n");
            				refresh();
					//printw("MOVING X: %d\n", detected_face.x);
            				//refresh();
					//printw("MOVING Y: %d\n", detected_face.y);
            				//refresh();
					//move(face);
				}
				else
				{
					if(moving == false && (abs(cmt.bb_rot.center.x - detected_face.x) > 30 || abs(cmt.bb_rot.center.y - detected_face.y) > 30))
					{
						moving = true;
						moving_face.x = cmt.bb_rot.center.x;
						moving_face.y = cmt.bb_rot.center.y;
						printw("moving = true\n");
            					refresh();
						//sleep(1);
						//cv::Point face(detected_face.x * 2.72, detected_face.y * 2.72);
						//move(face);
					}
					if (moving == true && (abs(cmt.bb_rot.center.x - moving_face.x) < 1 && abs(cmt.bb_rot.center.y - moving_face.y) < 1))
					{
						detected_face.x = moving_face.x;
						detected_face.y = moving_face.y;
						cv::Point face(moving_face.x * 3, moving_face.y * 2.25);
						//printw("MOVING X: %d\n", detected_face.x);
            					//refresh();
						//printw("MOVING Y: %d\n", detected_face.y);
            					//refresh();
						move(face);
						
						moving = false;
						printw("moving = false\n");
            					refresh();
					}
					if (moving == true)
					{
						moving_face.x = cmt.bb_rot.center.x;
						moving_face.y = cmt.bb_rot.center.y;						
						//detected_face.y = moving_face.y;
						
						//detected_face.x = 0; && (abs(moving_face.x - detected_face.x) > 100 || abs(moving_face.y - detected_face.y) > 100)
						//cout << " DETECTED X: " << detected_face.x << " DETETCTED Y: " << detected_face.y << endl;
					
					}*/
					/*else if (moving == true && abs(moving_face.x - cmt.bb_rot.center.x) < 2 && abs(moving_face.y - cmt.bb_rot.center.y) < 2)
					{
						moving == false;
						detected_face.x = moving_face.x;
						detected_face.y = moving_face.y;
						//cout << " DETECTED X: " << detected_face.x << " DETETCTED Y: " << detected_face.y << endl;
					}*/
				//}
		}
		
	}
}

void move(cv::Point point)
{
	bool pan = false;
	bool tilt = false;
	float moveX = 0;
	float moveY = 0;
	if(abs(point.x-border_x)>60)
	{
		pan = true;
		moveX = border_x - point.x;
	}
	if(abs(point.y-border_y)>60)
	{
		tilt = true;
		moveY = point.y - border_y;
	}
	/*printw("border x: %f\n", border_x);
            		refresh();
	printw("border y: %f\n", border_y);
            		refresh();
	printw("point X: %d\n", point.x);
            		refresh();
	printw("point Y: %d\n", point.y);
            		refresh();
	printw("move X: %f\n", moveX);
            		refresh();
	printw("move Y: %f\n", moveY);
            		refresh();*/
	
	if(pan==true||tilt==true)
	{
		
		//struct soap *soap = soap_new();	
		/*				
		_tptz__Stop *tptz__Stop = soap_new__tptz__Stop(soap, -1);
		_tptz__StopResponse *tptz__StopResponse = soap_new__tptz__StopResponse(soap, -1);
						
		tptz__Stop->ProfileToken = "Profile_1";
					
		if(SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
        	{		
  			printw("TOKEN ERROR\n");
            		refresh();
        	}
       		if(SOAP_OK == proxyPTZ.Stop(tptz__Stop, tptz__StopResponse))
		{
			printw("STOPPED\n");
            		refresh();
		}
						
		soap_destroy(soap);
		soap_end(soap);*/

		_tptz__RelativeMove *tptz__RelativeMove = soap_new__tptz__RelativeMove(soap, -1);
        	_tptz__RelativeMoveResponse *tptz__RelativeMoveResponse = soap_new__tptz__RelativeMoveResponse(soap, -1);

       		//tt__PTZSpeed * Speed = soap_new_tt__PTZSpeed(soap, -1);
        	//Speed->PanTilt = new tt__Vector2D;
        	//Speed->Zoom = new tt__Vector1D;
		tt__PTZVector *movement = soap_new_tt__PTZVector(soap, -1);
		movement->PanTilt = new tt__Vector2D;
		movement->Zoom = new tt__Vector1D;
		
		float angl_x, angl_y, fx, fy, f, f1;
		fx = 805.5;
		fy = 453.1;
		f = 1100.0;
		f1 = 320.0;

		if(abs(moveX)>0) 
		{
			angl_x = atan2(moveX,f1) * 180/PI;
			movement->PanTilt->x = angl_x/350;
		}
		if(abs(moveY)>0) 
		{
			angl_y = atan2(moveY,f) * 180/PI;
			movement->PanTilt->y = angl_y/90;
		}
		
		//printw("angl X: %f\n", angl_x);
            		//refresh();
		//printw("andl Y: %f\n", angl_y);
            		//refresh();
        	movement->Zoom->x = 0.0;

        	tptz__RelativeMove->Translation = movement;
        	tptz__RelativeMove->ProfileToken = "Profile_1";

		//LONG64 timeout = 1000;
		//if(moveX!=0){timeout = moveX/0.7;}
		//else {timeout = moveY/0.7;}
        	//tptz__ContinuousMove->Timeout = &timeout;
	

	/*
	Speed->PanTilt->x = 0.5;
	Speed->PanTilt->y = 0.0;
	Speed->Zoom->x = 0.0;

	tptz__ContinuousMove->Velocity = Speed;
	tptz__ContinuousMove->ProfileToken = "Profile_1";

	LONG64 timeout = 2000;
	tptz__ContinuousMove->Timeout = &timeout;
	*/		
		if(SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
        	{		
                	printw("TOKEN ERROR\n");
            		refresh();
        	}
       		if(SOAP_OK == proxyPTZ.RelativeMove(tptz__RelativeMove, tptz__RelativeMoveResponse))
        	{
                	printw("MOVED X: %f\n", moveX);
            		refresh();
			printw("MOVED Y: %f\n", moveY);
            		refresh();
			//sleep(8);
		}
        	soap_destroy(soap);
        	soap_end(soap);
	}
}
