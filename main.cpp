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
 
#include "include/soapDeviceBindingProxy.h"
#include "include/soapMediaBindingProxy.h"
#include "include/soapPTZBindingProxy.h"

#include "include/soapPullPointSubscriptionBindingProxy.h"
#include "include/soapRemoteDiscoveryBindingProxy.h" 

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

cv::String face_cascade_name = "haarcascade_frontalface_alt.xml";
cv::String profileface_cascade_name = "haarcascade_profileface.xml";
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier profile_cascade;
PTZBindingProxy proxyPTZ;
std::vector<cv::Rect> detected_faces;
cv::Point detected_face(0,0);
cv::Point moving_face(0, 0);
bool tracking = false;
bool moving = false;
cv::Rect rect;
cv::VideoCapture capture;

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
	cv::Mat frame;
	if(!face_cascade.load(face_cascade_name)){std::cout <<"Error loading face cascade!"<<std::endl; return -1;}
	if(!profile_cascade.load(profileface_cascade_name)){std::cout <<"Error loading profile cascade!"<<std::endl; return -1;}

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
    		std::cout  <<  "usage: ./ipconvif -cIp [<camera-ip>:<port>] -cUsr <cam-id> -cPwd <cam-pwd>\n";

    		return -1;
    	}

    	char *camIp = getCmdOption(argv, argv+argc, "-cIp");
   	char *camUsr = getCmdOption(argv, argv+argc, "-cUsr");
    	char *camPwd = getCmdOption(argv, argv+argc, "-cPwd");

    	if (!(camIp && camUsr && camPwd ))
    	{
    		processEventLog(__FILE__, __LINE__, stdout, "Error: Invalid args (All args are required!)");
    		return -1;
    	}

    	strcat(szHostName, "http://");
	strcat(szHostName, camIp);
	strcat(szHostName, ":10080/onvif/device_service");

	strcat(szPTZName, "http://");
	strcat(szPTZName, camIp);
	strcat(szPTZName, ":10080/onvif/PTZ");

	strcat(szStreamName, "rtsp://");
	strcat(szStreamName, camIp);
	strcat(szStreamName, ":10554//Streaming/Channels/2");

	proxyDevice.soap_endpoint = szHostName;
	proxyPTZ.soap_endpoint = szPTZName;

	// Register plugins
	soap_register_plugin(proxyDevice.soap, soap_wsse);
	soap_register_plugin(proxyPTZ.soap, soap_wsse);

	struct soap *soap = soap_new();
	// For DeviceBindingProxy
	if (SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyDevice.soap, NULL, camUsr, camPwd))
	{
		return -1;
	}

	if (SOAP_OK != soap_wsse_add_Timestamp(proxyDevice.soap, "Time", 10)) 
	{
		return -1;
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
    	}
    	else
    	{
        	PrintErr(proxyDevice.soap);
        	//std::cout << "Error getting device information"<<std::endl;
		return -1;
    	}
    
    	// DeviceBindingProxy ends
    	soap_destroy(soap); 
    	soap_end(soap); 
	
	capture.open(szStreamName);

	if(!capture.isOpened()){std::cout << "Error opening video capture!"<<std::endl; return -1;}

	while(1)
	{	
		capture.grab();
		capture.retrieve(im);	
		if(!frame.empty())
		{
			if( tracking == false)
			{
				detect(frame);
			}
			else
			{
				track(frame);
			}
		}
		else
		{
			std::cout <<"No captured frame!" << std::endl;
			capture.open(szStreamName);
		}
		//cv::waitKey(100);

	}
	return 0;
}

void detect(cv::Mat frame)
{
	std::vector<cv::Rect> faces;
	cv::Mat frame_gray;
	cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
	cv::equalizeHist(frame_gray,frame_gray);
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
					std::cout<<"Found a face"<<std::endl;
					//cv::Point face(faces[0].x*2.72 + faces[0].width*1.36, faces[0].y*1.875 + faces[0].height*0.9375);
					//detected_face = face;		
					tracking = true;
					rect = faces[0];
					//rect.height = rect.height - 20;
					//rect.width = rect.width - 30;
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
	cv::Mat frame0_gray, frame;
	cmt::CMT cmt;

	if (frame0.channels() > 1) {
		cvtColor(frame0, frame0_gray, CV_BGR2GRAY);
		equalizeHist(frame0_gray, frame0_gray);
	}
	else {
		frame0_gray = frame0;
	}
	cmt.initialize(frame0_gray, rect);

	while (tracking)
	{
		capture.grab();
		capture.retrieve(frame);
		if (frame.empty()) break;

		cv::Mat frame_gray;

		if (frame.channels() > 1) {
			cvtColor(frame, frame_gray, CV_BGR2GRAY);
			equalizeHist(frame_gray, frame_gray);
		}
		else {
			frame_gray = frame;
		}
		cmt.processFrame(frame_gray);


		//char key = display(frame, cmt);
		if ((abs(cmt.bb_rot.size.height - rect.height) > 20 || abs(cmt.bb_rot.size.width - rect.width) > 20) && (abs(detected_face.x - cmt.bb_rot.center.x) > 20 || abs(detected_face.y - cmt.bb_rot.center.y) > 20))
		{
			tracking = false;
			//break;
		}
		else
		{
				if(detected_face.x == 0)
				{
					detected_face.x = cmt.bb_rot.center.x;
					detected_face.y = cmt.bb_rot.center.y;
					cv::Point face(detected_face.x * 2.72, detected_face.y * 1.875);
					move(face);
				}
				else
				{
					if(abs(cmt.bb_rot.center.x - detected_face.x) > 10 || abs(cmt.bb_rot.center.y - detected_face.y) > 10)
					{
						moving = true;
						moving_face.x = cmt.bb_rot.center.x;
						moving_face.y = cmt.bb_rot.center.y;
						//cv::Point face(detected_face.x * 2.72, detected_face.y * 2.72);
						//move(face);
					}
					if (moving == true && (abs(moving_face.x - detected_face.x) > 100 || abs(moving_face.y - detected_face.y) > 100))
					{
						
						struct soap *soap = soap_new();	
						
						_tptz__Stop *tptz__Stop = soap_new__tptz__Stop(soap, -1);
						_tptz__StopResponse *tptz__StopResponse = soap_new__tptz__StopResponse(soap, -1);
						
						tptz__Stop->ProfileToken = "Profile_1";
						
						if(SOAP_OK != soap_wsse_add_UsernameTokenDigest(proxyPTZ.soap, NULL, "admin", "Supervisor"))
        					{		
                					std::cout << "ERROR" << std::endl;
        					}
       						if(SOAP_OK == proxyPTZ.Stop(tptz__Stop, tptz__StopResponse))
						{
							std::cout << "STOPPED" << std::endl;
						}
						
						soap_destroy(soap);
						soap_end(soap);
						
						detected_face.x = moving_face.x;
						detected_face.y = moving_face.y;
						Point face(detected_face.x * 2.72, detected_face.y * 2.72);
						move(face);
						//cout << " DETECTED X: " << detected_face.x << " DETETCTED Y: " << detected_face.y << endl;
					
					}
					else if (moving == true && abs(moving_face.x - cmt.bb_rot.center.x) < 2 && abs(moving_face.y - cmt.bb_rot.center.y) < 2)
					{
						moving == false;
						detected_face.x = moving_face.x;
						detected_face.y = moving_face.y;
						//cout << " DETECTED X: " << detected_face.x << " DETETCTED Y: " << detected_face.y << endl;
					}
				}
		}
		
	}
}

void move(cv::Point point)
{
	bool pan = false;
	bool tilt = false;
	float moveX = 0;
	float moveY = 0;
	if(point.x<900||point.x>1020)
	{
		pan = true;
		moveX = 960 - point.x;
	}
	if(point.y<480||point.y>600)
	{
		tilt = true;
		moveY =  point.y -540;
	}
	
	if(pan==true||tilt==true)
	{
		struct soap *soap = soap_new();	

		_tptz__RelativeMove *tptz__RelativeMove = soap_new__tptz__RelativeMove(soap, -1);
        	_tptz__RelativeMoveResponse *tptz__RelativeMoveResponse = soap_new__tptz__RelativeMoveResponse(soap, -1);

       		//tt__PTZSpeed * Speed = soap_new_tt__PTZSpeed(soap, -1);
        	//Speed->PanTilt = new tt__Vector2D;
        	//Speed->Zoom = new tt__Vector1D;
		tt__PTZVector *movement = soap_new_tt__PTZVector(soap, -1);
		movement->PanTilt = new tt__Vector2D;
		movement->Zoom = new tt__Vector1D;
		
		float angl_x, angl_y, fx, fy;
		fx = 805.5;
		fy = 453.1;

		if(abs(moveX)>0) 
		{
			angl_x = atan2(moveX,fx) * 180/PI;
			movement->PanTilt->x = angl_x/350;
		}
		if(abs(moveY)>0) 
		{
			angl_y = atan2(moveY,fy) * 180/PI;
			movement->PanTilt->y = angl_y/90;
		}
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
                	std::cout << "ERROR" << std::endl;
        	}
       		if(SOAP_OK == proxyPTZ.RelativeMove(tptz__RelativeMove, tptz__RelativeMoveResponse))
        	{
                	std::cout << "DONE" << std::endl;
			std::cout << moveX << std::endl;
			std::cout << moveY << std::endl;
			//sleep(8);
		}
        	soap_destroy(soap);
        	soap_end(soap);
	}
}
