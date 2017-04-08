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
#include "Snapshot.hpp"
#include <algorithm>
#include <cstring>

#define PI 3.14159265
#define MAX_HOSTNAME_LEN 128
#define MAX_LOGMSG_LEN 256 

using cmt::CMT;

struct soap *soap = soap_new();

cv::String face_cascade_name = "haarcascade_frontalface_alt.xml";
cv::String profileface_cascade_name = "haarcascade_profileface.xml";
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier profile_cascade;
PTZBindingProxy proxyPTZ;
std::vector<cv::Rect> detected_faces;
cv::Point detected_face(0.0,0.0);
cv::Point moving_face(0.0, 0.0);
bool tracking = false;
bool moving = false;
bool camera_control = false;
cv::Rect rect;
cv::VideoCapture capture;
float x = 0, y = 0;
float border_x = 960;
float border_y = 540;
char *fifo = "ipcfifo";
int fd;
char buf[1024];

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
		if (!capture.isOpened())
		{
			capture.open("rtsp://192.168.11.19:554//Streaming/Channels/2");
		}
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

void *ContMove(void *threadid)
{
	while(1)
	{
		printw("da\n");
		if (moving)
		{
			_tptz__ContinuousMove *tptz__ContinuousMove = soap_new__tptz__ContinuousMove(soap, -1);
			_tptz__ContinuousMoveResponse *tptz__ContinuousMoveResponse = soap_new__tptz__ContinuousMoveResponse(soap, -1);

			tt__PTZSpeed *Speed = soap_new_tt__PTZSpeed(soap, -1);
			
			Speed->PanTilt = new tt__Vector2D;
			Speed->Zoom = new tt__Vector1D;
			Speed->PanTilt->x = x;
			Speed->PanTilt->y = y;
			Speed->Zoom->x = 0.0;
			
			tptz__ContinuousMove->ProfileToken = "Profile_1";
			tptz__ContinuousMove->Velocity = Speed;
			
			if (SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
			{
				printw("TOKEN ERROR\n");
            			refresh();
			}
			soap_wsse_add_Timestamp(proxyPTZ.soap, "Time", 10);
			if (SOAP_OK == proxyPTZ.ContinuousMove(tptz__ContinuousMove, tptz__ContinuousMoveResponse))
			{
				printw("MOVED X: %f\n", tptz__ContinuousMove->Velocity->PanTilt->x);
            			refresh();
				printw("MOVED Y: %f\n", tptz__ContinuousMove->Velocity->PanTilt->y);
            			refresh();
			}
			soap_destroy(soap); 
    			soap_end(soap);
		}
	}
	pthread_exit(NULL);
	return NULL;
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
	
	char szHostName[MAX_HOSTNAME_LEN] = { 0 };
	char szPTZName[MAX_HOSTNAME_LEN] = {0};
	char szStreamName[MAX_HOSTNAME_LEN] = {0};
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

    	char *camIp = getCmdOption(argv, argv+argc, "-cIp");
   	char *camUsr = getCmdOption(argv, argv+argc, "-cUsr");
    	char *camPwd = getCmdOption(argv, argv+argc, "-cPwd");

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
	capture.grab();
	capture.retrieve(im);
	
	
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
				rect.y = 250;
				rect.height = 50;
				rect.width = 150;
			}
			else if(ch == 112)
			{
				printw("Moving to a specific point\n");
            			refresh();
				cv::Point face(0.0, 0.0);
				move(face);
			}
				
		}
		if(read(fd, buf, 1024))
		{
			if(*buf == 113)
			{
				printw("Exit\n");
            			refresh();
				endwin();
				return 0;
			}
			/*else if(*buf == 116)
			{
				printw("Tracking at specified spot\n");
            			refresh();
				tracking = true;
				rect.x = 200;
				rect.y = 250;
				rect.height = 50;
				rect.width = 150;
			}*/
			else if(*buf == 112)
			{
				printw("Moving to a specific point\n");
            			refresh();
				cv::Point face(0.0, 0.0);
				move(face);
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
	std::vector<cv::Rect> faces;
	cv::Mat frame_gray;
	cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
	//cv::equalizeHist(frame_gray,frame_gray);
	int i, dif;
	
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30,30));
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
					rect.height = rect.height - 10;
					rect.width = rect.width - 20;
					rect.x = rect.x + 10;
					rect.y = rect.y + 10;
					for(i;i>=0;i--){detected_faces.pop_back();}
				}
				else
				{
					detected_faces.push_back(faces[0]);
				}
			}
		}
	}
}

void track(cv::Mat frame0)
{
	int verbose_flag = 0;
	FILELog::ReportingLevel() = verbose_flag ? logDEBUG : logINFO;
	Output2FILE::Stream() = stdout; //Log to stdout
	cv::Mat frame0_gray;
	CMT cmt;

	if (frame0.channels() > 1) {
		cvtColor(frame0, frame0_gray, CV_BGR2GRAY);
		//equalizeHist(frame0_gray, frame0_gray);
	}
	else {
		frame0_gray = frame0;
	}
	cmt.initialize(frame0_gray, rect);

	while (tracking)
	{
		//capture.grab();
		//capture.retrieve(frame);
		//if (frame.empty()) break;

		cv::Mat frame_gray;

		if (im.channels() > 1) {
			cvtColor(im, frame_gray, CV_BGR2GRAY);
			//equalizeHist(frame_gray, frame_gray);
		}
		else {
			frame_gray = im;
		}
		cmt.processFrame(frame_gray);


		//char key = display(frame, cmt);
		if (control())
		{
           	 	int ch = getch();
			if(ch == 115)
			{
				printw("Stopped tracking\n");
            			refresh();
				tracking = false;
				detected_face.x = 0;
				break;
			}
			else if(ch == 99)
			{
				printw("Camera control changed\n");
            			refresh();
				camera_control = !camera_control;
			}
			else if(ch == 109)
			{
				printw("Center position\n");
            			refresh();
				detected_face.x == 0;
				border_x = 960;
				border_y = 540;
			}
			else if(ch == 108)
			{
				printw("Left position\n");
            			refresh();
				border_x = 640;
				border_y = 720;
				detected_face.x == 0;
			}
			else if(ch == 114)
			{
				printw("Right position\n");
            			refresh();
				border_x = 1280;
				border_y = 720;
				detected_face.x == 0;
			}
		}
		if(read(fd, buf, 1024))
		{
			if(*buf == 115)
			{
				printw("Stopped tracking\n");
            			refresh();
				tracking = false;
				detected_face.x = 0;
				break;
			}
			else if(*buf == 99)
			{
				printw("Camera control changed\n");
            			refresh();
				camera_control = !camera_control;
			}
		}
		if ((abs(cmt.bb_rot.size.height - rect.height) > 50 || abs(cmt.bb_rot.size.width - rect.width) > 50)/*&& (abs(detected_face.x - cmt.bb_rot.center.x) > 20 || abs(detected_face.y - cmt.bb_rot.center.y) > 20)*/)
		{
			printw("Stopped tracking\n");
            		refresh();
			tracking = false;
			detected_face.x = 0;
			//break;
		}
		else if(camera_control)
		{
			if(abs(cmt.bb_rot.center.x - 320) > 30 || abs(cmt.bb_rot.center.y - 240) > 25)
			{
				printw("yes\n");
				if((cmt.bb_rot.center.x - 320)/1000 > 0.03)
				{
					x = (cmt.bb_rot.center.x - 320)/1000 + 0.1;
				}
				else if ((cmt.bb_rot.center.x - 320)/1000 < -0.03)
				{
					x = (cmt.bb_rot.center.x - 320)/1000 - 0.1;
				}
				else
				{
					x = 0;
				}
				if(abs(cmt.bb_rot.center.y - 240) > 0.025)
				{
					y = abs(cmt.bb_rot.center.y - 240) + 0.1;
				}
				else if (abs(cmt.bb_rot.center.y - 240) < -0.025)
				{
					y = abs(cmt.bb_rot.center.y - 240) - 0.1;
				}
				else
				{
					y = 0;
				}
				moving = true;
			}
			else if (moving)
			{
				moving = false;
				_tptz__Stop *tptz__Stop = soap_new__tptz__Stop(soap, -1);
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
				soap_end(soap);
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
