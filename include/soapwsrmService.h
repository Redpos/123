/* soapwsrmService.h
   Generated by gSOAP 2.8.17r from onvif.h

Copyright(C) 2000-2013, Robert van Engelen, Genivia Inc. All Rights Reserved.
The generated code is released under one of the following licenses:
GPL or Genivia's license for commercial use.
This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
*/

#ifndef soapwsrmService_H
#define soapwsrmService_H
#include "soapH.h"
class SOAP_CMAC wsrmService
{ public:
	struct soap *soap;
	bool own;
	/// Constructor
	wsrmService();
	/// Constructor to use/share an engine state
	wsrmService(struct soap*);
	/// Constructor with engine input+output mode control
	wsrmService(soap_mode iomode);
	/// Constructor with engine input and output mode control
	wsrmService(soap_mode imode, soap_mode omode);
	/// Destructor, also frees all deserialized data
	virtual ~wsrmService();
	/// Delete all deserialized data (with soap_destroy and soap_end)
	virtual	void destroy();
	/// Delete all deserialized data and reset to defaults
	virtual	void reset();
	/// Initializer used by constructor
	virtual	void wsrmService_init(soap_mode imode, soap_mode omode);
	/// Create a copy
	virtual	wsrmService *copy() SOAP_PURE_VIRTUAL;
	/// Close connection (normally automatic)
	virtual	int soap_close_socket();
	/// Force close connection (can kill a thread blocked on IO)
	virtual	int soap_force_close_socket();
	/// Return sender-related fault to sender
	virtual	int soap_senderfault(const char *string, const char *detailXML);
	/// Return sender-related fault with SOAP 1.2 subcode to sender
	virtual	int soap_senderfault(const char *subcodeQName, const char *string, const char *detailXML);
	/// Return receiver-related fault to sender
	virtual	int soap_receiverfault(const char *string, const char *detailXML);
	/// Return receiver-related fault with SOAP 1.2 subcode to sender
	virtual	int soap_receiverfault(const char *subcodeQName, const char *string, const char *detailXML);
	/// Print fault
	virtual	void soap_print_fault(FILE*);
#ifndef WITH_LEAN
	/// Print fault to stream
#ifndef WITH_COMPAT
	virtual	void soap_stream_fault(std::ostream&);
#endif
	/// Put fault into buffer
	virtual	char *soap_sprint_fault(char *buf, size_t len);
#endif
	/// Disables and removes SOAP Header from message
	virtual	void soap_noheader();
	/// Put SOAP Header in message
	virtual	void soap_header(char *wsa__MessageID, struct wsa__Relationship *wsa__RelatesTo, struct wsa__EndpointReferenceType *wsa__From, struct wsa__EndpointReferenceType *wsa__ReplyTo, struct wsa__EndpointReferenceType *wsa__FaultTo, char *wsa__To, char *wsa__Action, struct wsdd__AppSequenceType *wsdd__AppSequence, char *wsa5__MessageID, struct wsa5__RelatesToType *wsa5__RelatesTo, struct wsa5__EndpointReferenceType *wsa5__From, struct wsa5__EndpointReferenceType *wsa5__ReplyTo, struct wsa5__EndpointReferenceType *wsa5__FaultTo, char *wsa5__To, char *wsa5__Action, struct chan__ChannelInstanceType *chan__ChannelInstance, struct wsrm__SequenceType *wsrm__Sequence, int __sizeAckRequested, struct wsrm__AckRequestedType *wsrm__AckRequested, int __sizeSequenceAcknowledgement, struct _wsrm__SequenceAcknowledgement *wsrm__SequenceAcknowledgement, struct wsrm__SequenceFaultType *wsrm__SequenceFault, struct _wsse__Security *wsse__Security);
	/// Get SOAP Header structure (NULL when absent)
	virtual	const SOAP_ENV__Header *soap_header();
	/// Run simple single-thread iterative service on port until a connection error occurs (returns error code or SOAP_OK), use this->bind_flag = SO_REUSEADDR to rebind for a rerun
	virtual	int run(int port);
	/// Bind service to port (returns master socket or SOAP_INVALID_SOCKET)
	virtual	SOAP_SOCKET bind(const char *host, int port, int backlog);
	/// Accept next request (returns socket or SOAP_INVALID_SOCKET)
	virtual	SOAP_SOCKET accept();
#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)
	/// Then accept SSL handshake, when SSL is used
	virtual	int ssl_accept();
#endif
	/// Serve this request (returns error code or SOAP_OK)
	virtual	int serve();
	/// Used by serve() to dispatch a request (returns error code or SOAP_OK)
	virtual	int dispatch();

	///
	/// Service operations (you should define these):
	/// Note: compile with -DWITH_PURE_VIRTUAL for pure virtual methods
	///

	/// Web service operation 'CreateSequence' (returns error code or SOAP_OK)
	virtual	int CreateSequence(struct wsrm__CreateSequenceType *wsrm__CreateSequence, struct wsrm__CreateSequenceResponseType *wsrm__CreateSequenceResponse) SOAP_PURE_VIRTUAL;

	/// Web service operation 'CloseSequence' (returns error code or SOAP_OK)
	virtual	int CloseSequence(struct wsrm__CloseSequenceType *wsrm__CloseSequence, struct wsrm__CloseSequenceResponseType *wsrm__CloseSequenceResponse) SOAP_PURE_VIRTUAL;

	/// Web service operation 'TerminateSequence' (returns error code or SOAP_OK)
	virtual	int TerminateSequence(struct wsrm__TerminateSequenceType *wsrm__TerminateSequence, struct wsrm__TerminateSequenceResponseType *wsrm__TerminateSequenceResponse) SOAP_PURE_VIRTUAL;

	/// Web service one-way operation 'CreateSequenceResponse' (return error code, SOAP_OK (no response), or send_CreateSequenceResponse_empty_response())
	virtual	int CreateSequenceResponse(struct wsrm__CreateSequenceResponseType *wsrm__CreateSequenceResponse) SOAP_PURE_VIRTUAL;
	virtual	int send_CreateSequenceResponse_empty_response(int httpcode) { return soap_send_empty_response(this->soap, httpcode); }

	/// Web service one-way operation 'CloseSequenceResponse' (return error code, SOAP_OK (no response), or send_CloseSequenceResponse_empty_response())
	virtual	int CloseSequenceResponse(struct wsrm__CloseSequenceResponseType *wsrm__CloseSequenceResponse) SOAP_PURE_VIRTUAL;
	virtual	int send_CloseSequenceResponse_empty_response(int httpcode) { return soap_send_empty_response(this->soap, httpcode); }

	/// Web service one-way operation 'TerminateSequenceResponse' (return error code, SOAP_OK (no response), or send_TerminateSequenceResponse_empty_response())
	virtual	int TerminateSequenceResponse(struct wsrm__TerminateSequenceResponseType *wsrm__TerminateSequenceResponse) SOAP_PURE_VIRTUAL;
	virtual	int send_TerminateSequenceResponse_empty_response(int httpcode) { return soap_send_empty_response(this->soap, httpcode); }

	/// Web service one-way operation 'SequenceAcknowledgement' (return error code, SOAP_OK (no response), or send_SequenceAcknowledgement_empty_response())
	virtual	int SequenceAcknowledgement() SOAP_PURE_VIRTUAL;
	virtual	int send_SequenceAcknowledgement_empty_response(int httpcode) { return soap_send_empty_response(this->soap, httpcode); }

	/// Web service one-way operation 'AckRequested' (return error code, SOAP_OK (no response), or send_AckRequested_empty_response())
	virtual	int AckRequested() SOAP_PURE_VIRTUAL;
	virtual	int send_AckRequested_empty_response(int httpcode) { return soap_send_empty_response(this->soap, httpcode); }
};
#endif