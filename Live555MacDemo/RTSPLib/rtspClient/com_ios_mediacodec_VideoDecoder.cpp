//
//  com_ios_mediacodec_VideoDecoder.cpp
//  RTSPBasedPlayer
//
//  Created by thundersoft on 16/5/25.
//  Copyright © 2016年 thundersoft. All rights reserved.
//

#include "com_ios_mediacodec_VideoDecoder.hh"


//#define LOG_NDEBUG 0
#define LOG_TAG "IPC_DEV"
//#include "utils/Log.h"

//#include <JNIHelp.h>
//#include <android_runtime/AndroidRuntime.h>
//#include <media/stagefright/foundation/ADebug.h>
#include <pthread.h>

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "H264VideoRTPSource.hh"
#include "OutputFile.hh"
#include<fcntl.h>
#include <stdlib.h>
//#include <jni.h>

//using namespace android;
#include "queue.h"


static playStatus pStatus;

uint8_t buffer[100] = { 0 };

static RTSPClient* rtspClient;

//static void* main_setup_rtsp(void* rv);
void init();
//static JNIEnv *GetEnv();
//static void DetachCurrent();
//void notify(int msg, const jbyteArray cbb, long presentationTime);

// Forward function definitions:
// RTSP 'response handlers':
void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode,char* resultString);
void continueAfterSETUP(RTSPClient* rtspClient, int resultCode,char* resultString);
void continueAfterPLAY(RTSPClient* rtspClient, int resultCode,char* resultString);

// Other event handler functions:
void subsessionAfterPlaying(void* clientData); // called when a stream's subsession (e.g., audio or video substream) ends
void subsessionByeHandler(void* clientData); // called when a RTCP "BYE" is received for a subsession

void subsessionAppHandler(void* clientData,
                          u_int8_t subtype, u_int32_t nameBytes/*big-endian order*/,
                          u_int8_t* appDependentData, unsigned appDependentDataSize);

void streamTimerHandler(void* clientData);
// called at the end of a stream's expected duration (if the stream has not already signaled its end using a RTCP "BYE")

// The main streaming routine (for each "rtsp://" URL):
void openURL(UsageEnvironment& env, char const* progName, char const* rtspURL);
// Used to iterate through each stream's 'subsessions', setting up each one:
void setupNextSubsession(RTSPClient* rtspClient);
// Used to shut down and close a stream (including its "RTSPClient" object):
void shutdownStream(RTSPClient* rtspClient, int exitCode = 1);

// A function that outputs a string that identifies each stream (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env,const RTSPClient& rtspClient) {
    return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env,const MediaSubsession& subsession) {
    return env << subsession.mediumName() << "/" << subsession.codecName();
}

void usage(UsageEnvironment& env, char const* progName) {
    env << "Usage: " << progName << " <rtsp-url-1> ... <rtsp-url-N>\n";
    env << "\t(where each <rtsp-url-i> is a \"rtsp://\" URL)\n";
}

char eventLoopWatchVariable = 0;
// Define a class to hold per-stream state that we maintain throughout each stream's lifetime:
class StreamClientState {
public:
    StreamClientState();
    virtual ~StreamClientState();
    
public:
    MediaSubsessionIterator* iter;
    MediaSession* session;
    MediaSubsession* subsession;
    TaskToken streamTimerTask;
    double duration;
};


class ourRTSPClient: public RTSPClient {
public:
    static ourRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL,
                                    int verbosityLevel = 0, char const* applicationName = NULL,
                                    portNumBits tunnelOverHTTPPortNum = 0);
    
protected:
    ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
                  int verbosityLevel, char const* applicationName,
                  portNumBits tunnelOverHTTPPortNum);
    // called only by createNew();
    virtual ~ourRTSPClient();
    
public:
    StreamClientState scs;
};


//#define FILE_SAVE_ENB
#define CHECK_IDRFRM_ENB
class HEVCSink: public MediaSink {
public:
    static HEVCSink* createNew(UsageEnvironment& env,
                               MediaSubsession& subsession, // identifies the kind of data that's being received
                               char const* streamId = NULL); // identifies the stream itself (optional)
    
private:
    HEVCSink(UsageEnvironment& env, MediaSubsession& subsession,char const* streamId);
    // called only by "createNew()"
    virtual ~HEVCSink();
    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                  unsigned numTruncatedBytes, struct timeval presentationTime,
                                  unsigned durationInMicroseconds);
    virtual void afterGettingFrame(unsigned frameSize,
                                   unsigned numTruncatedBytes, struct timeval presentationTime);
    void addData(unsigned char const* data, unsigned dataSize);
    
private:
    // redefined virtual functions:
    virtual Boolean continuePlaying();
    char const* fSPropParameterSetsStr[3];
    Boolean fHaveWrittenFirstFrame;
    unsigned char* fBuffer;
    //unsigned fBufferSize;
    struct timeval fPrevPresentationTime;
    //unsigned fSamePresentationTimeCounter;
#ifdef FILE_SAVE_ENB
    //FILE* fOutFid;
#endif /* FILE_SAVE_ENB */
    
private:
    //u_int8_t* fReceiveBuffer;
    MediaSubsession& fSubsession;
#ifdef CHECK_IDRFRM_ENB
    char codecName[16];
#endif /* CHECK_IDRFRM_ENB */
    char* fStreamId;
#ifdef FILE_SAVE_ENB
    int fd;
#endif /* FILE_SAVE_ENB */
};

#define RTSP_CLIENT_VERBOSITY_LEVEL 1 // by default, print verbose output from each "RTSPClient"
static unsigned rtspClientCount = 0; // Counts how many streams (i.e., "RTSPClient"s) are currently in use.

void openURL(UsageEnvironment& env, char const* progName, char const* rtspURL) {
    // Begin by creating a "RTSPClient" object.  Note that there is a separate "RTSPClient" object for each stream that we wish
    // to receive (even if more than stream uses the same "rtsp://" URL).
    rtspClient = ourRTSPClient::createNew(env, rtspURL,
                                                      RTSP_CLIENT_VERBOSITY_LEVEL, progName);

    if (rtspClient == NULL) {
        env << "Failed to create a RTSP client for URL \"" << rtspURL << "\": "
        << env.getResultMsg() << "\n";
        //ALOGE("Failed to create a RTSP client for URL progName = %s, rtspURL= %s\n",progName,rtspURL);
        return;
    }
    //ALOGE("rtspClient !=NULL ");
    if (pStatus) {
        rtspClient->getRtspClientStatus(pStatus);
    }
    ++rtspClientCount;
    
    // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the stream.
    // Note that this command - like all RTSP commands - is sent asynchronously; we do not block, waiting for a response.
    // Instead, the following function call returns immediately, and we handle the RTSP response later, from within the event loop:
    //ALOGE("continueAfterDESCRIBE BEGIN ");
    rtspClient->sendDescribeCommand(continueAfterDESCRIBE);
    //ALOGE("continueAfterDESCRIBE END ");
}

// Implementation of the RTSP 'response handlers':

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode,
                           char* resultString) {
    do {
        
        UsageEnvironment& env = rtspClient->envir(); // alias
        StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias
        if (resultCode != 0) {
            env << *rtspClient << "Failed to get a SDP description: "
            << resultString << "\n";
            delete[] resultString;
            break;
        }
        char* const sdpDescription = resultString;
        env << *rtspClient << "Got a SDP description:\n" << sdpDescription
        << "\n";
        
        // Create a media session object from this SDP description:
        scs.session = MediaSession::createNew(env, sdpDescription);
        delete[] sdpDescription; // because we don't need it anymore
        if (scs.session == NULL) {
            env << *rtspClient
            << "Failed to create a MediaSession object from the SDP description: "
            << env.getResultMsg() << "\n";
            break;
        } else if (!scs.session->hasSubsessions()) {
            env << *rtspClient
            << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
            break;
        }
        
        // Then, create and set up our data source objects for the session.  We do this by iterating over the session's 'subsessions',
        // calling "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command, on each one.
        // (Each 'subsession' will have its own data source.)
        scs.iter = new MediaSubsessionIterator(*scs.session);
        setupNextSubsession(rtspClient);
        return;
    } while (0);
    // An unrecoverable error occurred with this stream.
    shutdownStream(rtspClient);
}

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP-over-TCP, change the following to True:
#define REQUEST_STREAMING_OVER_TCP False
//#define REQUEST_STREAMING_OVER_TCP True

void setupNextSubsession(RTSPClient* rtspClient) {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias
    //ALOGE("setupNextSubsession 1 ");
    scs.subsession = scs.iter->next();
    if (scs.subsession != NULL) {
        if (!scs.subsession->initiate()) {
            //ALOGE("setupNextSubsession 2 ");
            env << *rtspClient << "Failed to initiate the \"" << *scs.subsession
            << "\" subsession: " << env.getResultMsg() << "\n";
            setupNextSubsession(rtspClient); // give up on this subsession; go to the next one
        } else {
            env << *rtspClient << "Initiated the \"" << *scs.subsession
            << "\" subsession (";
            if (scs.subsession->rtcpIsMuxed()) {
                env << "client port " << scs.subsession->clientPortNum();
            } else {
                env << "client ports " << scs.subsession->clientPortNum() << "-"
                << scs.subsession->clientPortNum() + 1;
            }
            env << ")\n";
            
            // Continue setting up this subsession, by sending a RTSP "SETUP" command:
            //ALOGE("setupNextSubsession 3 ");
            rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP,
                                         False, REQUEST_STREAMING_OVER_TCP);
        }
        return;
    }
    
    // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY" command to start the streaming:
    if (scs.session->absStartTime() != NULL) {
        // Special case: The stream is indexed by 'absolute' time, so send an appropriate "PLAY" command:
        rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY,
                                    scs.session->absStartTime(), scs.session->absEndTime());
    } else {
        scs.duration = scs.session->playEndTime()
        - scs.session->playStartTime();
        rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
    }
}

void continueAfterSETUP(RTSPClient* rtspClient, int resultCode,
                        char* resultString) {
    do {
        UsageEnvironment& env = rtspClient->envir(); // alias
        StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias
        
        if (resultCode != 0) {
            env << *rtspClient << "Failed to set up the \"" << *scs.subsession
            << "\" subsession: " << resultString << "\n";
            break;
        }
        
        env << *rtspClient << "Set up the \"" << *scs.subsession
        << "\" subsession (";
        if (scs.subsession->rtcpIsMuxed()) {
            env << "client port " << scs.subsession->clientPortNum();
        } else {
            env << "client ports " << scs.subsession->clientPortNum() << "-"
            << scs.subsession->clientPortNum() + 1;
        }
        env << ")\n";
        
        // Having successfully setup the subsession, create a data sink for it, and call "startPlaying()" on it.
        // (This will prepare the data sink to receive data; the actual flow of data from the client won't start happening until later,
        // after we've sent a RTSP "PLAY" command.)
        
        //scs.subsession->sink = DummySink::createNew(env, *scs.subsession, rtspClient->url());
//        scs.subsession->sink = HEVCSink::createNew(env, *scs.subsession,
//                                                   rtspClient->url());
        
        char buffer[256];
        
        //HOME is the home directory of your application
        //points to the root of your sandbox
        strcpy(buffer,getenv("HOME"));
        
        //concatenating the path string returned from HOME
        strcat(buffer,"/Documents/myfile.265");
        scs.subsession->sink = H264VideoFileSink::createNew(env, buffer);
        // perhaps use your own custom "MediaSink" subclass instead
        if (scs.subsession->sink == NULL) {
            env << *rtspClient << "Failed to create a data sink for the \""
            << *scs.subsession << "\" subsession: "
            << env.getResultMsg() << "\n";
            break;
        }
        
        env << *rtspClient << "Created a data sink for the \""
        << *scs.subsession << "\" subsession\n";
        scs.subsession->miscPtr = rtspClient; // a hack to let subsession handle functions get the "RTSPClient" from the subsession
        scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
                                           subsessionAfterPlaying, scs.subsession);
        // Also set a handler to be called if a RTCP "BYE" arrives for this subsession:
        if (scs.subsession->rtcpInstance() != NULL) {
            scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler,
                                                          scs.subsession);
            
//            scs.subsession->rtcpInstance()->setAppHandler(subsessionAppHandler,
//                                                          scs.subsession);
        }
    } while (0);
    delete[] resultString;
    
    // Set up the next subsession, if any:
    setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient* rtspClient, int resultCode,
                       char* resultString) {
    Boolean success = False;
    
    do {
        UsageEnvironment& env = rtspClient->envir(); // alias
        StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias
        
        if (resultCode != 0) {
            pStatus(RTSP_PLAYER_PLAY_ERROR);
            env << *rtspClient << "Failed to start playing session: "
            << resultString << "\n";
            break;
        }
        
        // Set a timer to be handled at the end of the stream's expected duration (if the stream does not already signal its end
        // using a RTCP "BYE").  This is optional.  If, instead, you want to keep the stream active - e.g., so you can later
        // 'seek' back within it and do another RTSP "PLAY" - then you can omit this code.
        // (Alternatively, if you don't want to receive the entire stream, you could set this timer for some shorter value.)
        if (scs.duration > 0) {
            unsigned const delaySlop = 2; // number of seconds extra to delay, after the stream's expected duration.  (This is optional.)
            scs.duration += delaySlop;
            unsigned uSecsToDelay = (unsigned) (scs.duration * 1000000);
            scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(
                                                                          uSecsToDelay, (TaskFunc*) streamTimerHandler, rtspClient);
        }
        
        env << *rtspClient << "Started playing session";
        if (scs.duration > 0) {
            env << " (for up to " << scs.duration << " seconds)";
        }
        env << "...\n";
        
        success = True;
        
    } while (0);
    delete[] resultString;
    
    if (!success) {
        // An unrecoverable error occurred with this stream.
        shutdownStream(rtspClient);
        pStatus(RTSP_PLAYER_PLAY_ERROR);
    }
    else
    {
        pStatus(RTSP_PLAYER_PLAY);
    }
}

// Implementation of the other event handlers:

void subsessionAfterPlaying(void* clientData) {
    MediaSubsession* subsession = (MediaSubsession*) clientData;
    RTSPClient* rtspClient = (RTSPClient*) (subsession->miscPtr);
    
    // Begin by closing this subsession's stream:
    Medium::close(subsession->sink);
    subsession->sink = NULL;
    
    // Next, check whether *all* subsessions' streams have now been closed:
    MediaSession& session = subsession->parentSession();
    MediaSubsessionIterator iter(session);
    while ((subsession = iter.next()) != NULL) {
        if (subsession->sink != NULL)
            return; // this subsession is still active
    }
    
    // All subsessions' streams have now been closed, so shutdown the client:
    shutdownStream(rtspClient);
}

void subsessionByeHandler(void* clientData) {
    MediaSubsession* subsession = (MediaSubsession*) clientData;
    RTSPClient* rtspClient = (RTSPClient*) subsession->miscPtr;
    UsageEnvironment& env = rtspClient->envir(); // alias
    
    env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession
    << "\" subsession\n";
    if(pStatus)
    {
        pStatus(RTSP_PLAYER_PLAY_STOPPED);
    }
    // Now act as if the subsession had closed:
    subsessionAfterPlaying(subsession);
}


void subsessionAppHandler(void* clientData,
                          u_int8_t subtype, u_int32_t nameBytes/*big-endian order*/,
                          u_int8_t* appDependentData, unsigned appDependentDataSize) {
    MediaSubsession* subsession = (MediaSubsession*) clientData;
    RTSPClient* rtspClient = (RTSPClient*) subsession->miscPtr;
    UsageEnvironment& env = rtspClient->envir(); // alias
    
    env << *rtspClient << "Received RTCP \"app\" on \"" << *subsession
    << "\" subsession\n";
    
    long long frameFlag0 = appDependentData[0];
    long long frameFlag1 = appDependentData[1];
    long long frameFlag2 = appDependentData[2];
    long long frameFlag3 = appDependentData[3];
    
    frameFlag3 = frameFlag3|(frameFlag2<<8)|(frameFlag1<<16)|(frameFlag0<<24);
    
//    debug_info("\n\r-------------------%08llx\n\r",frameFlag3);
//    for (int i =  0; i < appDependentDataSize; i ++) {
//        debug_info("%02x", appDependentData[i]);
//    }
//    debug_info("\n\r-----------------------\n\r");

    // Now act as if the subsession had closed:
//    subsessionAfterPlaying(subsession);
}


void streamTimerHandler(void* clientData) {
    ourRTSPClient* rtspClient = (ourRTSPClient*) clientData;
    StreamClientState& scs = rtspClient->scs; // alias
    
    scs.streamTimerTask = NULL;
    
    // Shut down the stream:
    shutdownStream(rtspClient);
}
/*
 extern void eixt_Notify();
 */
void shutdownStream(RTSPClient* rtspClient, int exitCode) {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias
    
    // First, check whether any subsessions have still to be closed:
    if (scs.session != NULL) {
        Boolean someSubsessionsWereActive = False;
        MediaSubsessionIterator iter(*scs.session);
        MediaSubsession* subsession;
        
        while ((subsession = iter.next()) != NULL) {
            if (subsession->sink != NULL) {
                Medium::close(subsession->sink);
                subsession->sink = NULL;
                
                if (subsession->rtcpInstance() != NULL) {
                    subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"
                }
                
                someSubsessionsWereActive = True;
            }
        }
        
        if (someSubsessionsWereActive) {
            // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the stream.
            // Don't bother handling the response to the "TEARDOWN".
            rtspClient->sendTeardownCommand(*scs.session, NULL);
        }
    }
    
    env << *rtspClient << "Closing the stream.\n";
    Medium::close(rtspClient);
    // Note that this will also cause this stream's "StreamClientState" structure to get reclaimed.
    
    if (--rtspClientCount == 0) {
        // The final stream has ended, so exit the application now.
        // (Of course, if you're embedding this code into your own application, you might want to comment this out,
        // and replace it with "eventLoopWatchVariable = 1;", so that we leave the LIVE555 event loop, and continue running "main()".)
        
        
        eventLoopWatchVariable = 1;
        sleep(1.0f);
        debug_info("final stream ended ,exit the application");
//        exit(exitCode);
        /*
         eixt_Notify();
         */
    }
}
// Implementation of "ourRTSPClient":

ourRTSPClient* ourRTSPClient::createNew(UsageEnvironment& env,
                                        char const* rtspURL, int verbosityLevel, char const* applicationName,
                                        portNumBits tunnelOverHTTPPortNum) {
    return new ourRTSPClient(env, rtspURL, verbosityLevel, applicationName,
                             tunnelOverHTTPPortNum);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
                             int verbosityLevel, char const* applicationName,
                             portNumBits tunnelOverHTTPPortNum) :
RTSPClient(env, rtspURL, verbosityLevel, applicationName,
           tunnelOverHTTPPortNum, -1) {
}

ourRTSPClient::~ourRTSPClient() {
}

// Implementation of "StreamClientState":

StreamClientState::StreamClientState() :
iter(NULL), session(NULL), subsession(NULL), streamTimerTask(NULL), duration(
                                                                             0.0) {
}

StreamClientState::~StreamClientState() {
    delete iter;
    if (session != NULL) {
        // We also need to delete "session", and unschedule "streamTimerTask" (if set)
        UsageEnvironment& env = session->envir(); // alias
        
        env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
        Medium::close(session);
    }
}

//// Implementation of "HEVCSink":
#define HEVC_SINK_RECEIVE_BUFFER_SIZE 500000

HEVCSink* HEVCSink::createNew(UsageEnvironment& env,
                              MediaSubsession& subsession, char const* streamId) {
    return new HEVCSink(env, subsession, streamId);
}

HEVCSink::HEVCSink(UsageEnvironment& env, MediaSubsession& subsession,
                   char const* streamId) :
MediaSink(env), fHaveWrittenFirstFrame(False), fSubsession(subsession) {
    fPrevPresentationTime.tv_sec = ~0;
    fPrevPresentationTime.tv_usec = 0;
    fBuffer = new unsigned char[HEVC_SINK_RECEIVE_BUFFER_SIZE];
    fStreamId = strDup(streamId);
#ifdef FILE_SAVE_ENB
    fd = open("/sdcard/jni-recv.265",O_CREAT | O_WRONLY | O_TRUNC,655);
    
    //fOutFid = OpenOutputFile(envir(), "rev.265");
#endif /* FILE_SAVE_ENB */
    if (strcmp(subsession.codecName(), "H264") == 0) {
        fSPropParameterSetsStr[0] = subsession.fmtp_spropparametersets();
        fSPropParameterSetsStr[1] = NULL;
        fSPropParameterSetsStr[2] = NULL;
#ifdef CHECK_IDRFRM_ENB
        snprintf(codecName, 16, "%s", "H264");
#endif /* CHECK_IDRFRM_ENB */
    } else if (strcmp(subsession.codecName(), "H265") == 0) {
        fSPropParameterSetsStr[0] = fSubsession.fmtp_spropvps();
        fSPropParameterSetsStr[1] = fSubsession.fmtp_spropsps();
        fSPropParameterSetsStr[2] = fSubsession.fmtp_sproppps();
#ifdef CHECK_IDRFRM_ENB
        snprintf(codecName, 16, "%s", "H265");
#endif /* CHECK_IDRFRM_ENB */
    } else {
        //ALOGE("HEVCSink::HEVCSink unsupport codec\n");
#ifdef CHECK_IDRFRM_ENB
        codecName[0] = '\0';
#endif /* CHECK_IDRFRM_ENB */
    }
}

HEVCSink::~HEVCSink() {
#ifdef FILE_SAVE_ENB
    //if( (NULL != fOutFid) ){
    //fclose(fOutFid);
    //}
    if( (fd > 0) ) {::close(fd);}
#endif /* FILE_SAVE_ENB */
    delete[] fBuffer;
    free(fStreamId);
}

void HEVCSink::afterGettingFrame(void* clientData, unsigned frameSize,
                                 unsigned numTruncatedBytes, struct timeval presentationTime,
                                 unsigned /*durationInMicroseconds*/) {
    HEVCSink* sink = (HEVCSink*) clientData;
    sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime);
}

// If you don't want to see debugging output for each received frame, then comment out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 1

void HEVCSink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                                 struct timeval presentationTime) {
    
    unsigned char const start_code[4] = { 0x00, 0x00, 0x00, 0x01 };
    
    // We've just received a frame of data.  (Optionally) print out information about it:
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
    if (fStreamId != NULL)
        envir() << "Stream \"" << fStreamId << "\"; ";
    debug_info("afterGettingFrame frameSize = %d\n\r",frameSize);
    if (strcmp(fSubsession.mediumName(), "video") != 0) {
        continuePlaying();
        return;
    }
    envir() << fSubsession.mediumName() << "/" << fSubsession.codecName()
    << ":\tReceived " << frameSize << " bytes";
    if (numTruncatedBytes > 0)
        envir() << " (with " << numTruncatedBytes << " bytes truncated)";
    char uSecsStr[6 + 1]; // used to output the 'microseconds' part of the presentation time
    sprintf(uSecsStr, "%06u", (unsigned) presentationTime.tv_usec);
    envir() << ".\tPresentation time: " << (int) presentationTime.tv_sec << "."
    << uSecsStr;
    debug_info("afterGettingFrame presentationTime = %d\n",(int)presentationTime.tv_sec);
    if (fSubsession.rtpSource() != NULL
        && !fSubsession.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
        envir() << "!"; // mark the debugging output to indicate that this presentation time is not RTCP-synchronized
    }
#ifdef DEBUG_PRINT_NPT
    envir() << "\tNPT: " << fSubsession.getNormalPlayTime(presentationTime);
#endif
    envir() << "\n";
#endif
    
#ifdef CHECK_IDRFRM_ENB
    if (!fHaveWrittenFirstFrame) {
        if (!strncmp(codecName, "H264", strlen("H264"))) {
            if ((fBuffer[0] != 0x65)) {
                goto SKIP_FRAME;
            }
        } else if (!strncmp(codecName, "H265", strlen("H265"))) {
            if ((fBuffer[0] != 0x26)) {
                goto SKIP_FRAME;
            }
        }
    }
#endif /* CHECK_IDRFRM_ENB */
    
    if ((fBuffer != NULL)) {
        // add vps sps pps
        unsigned sumByte = 0, writeByte = 0;
        unsigned numSPropRecords[3] = { 0, 0, 0 };
        SPropRecord* sPropRecords[3] = { NULL, NULL, NULL };
        //debug_info("afterGettingFrame fReceiveBuffer != NULL");
        if (!fHaveWrittenFirstFrame) {
            // If we have NAL units encoded in "sprop parameter strings", prepend these to the file:
            for (unsigned j = 0; j < 3; ++j) {
                sPropRecords[j] = parseSPropParameterSets(
                                                          fSPropParameterSetsStr[j], numSPropRecords[j]);
                for (unsigned i = 0; i < numSPropRecords[j]; ++i) {
                    sumByte += (4 + sPropRecords[j][i].sPropLength);
                }
            }
        }

        sumByte += (4 + frameSize);
        debug_info("NewByteArray frameSize = %d\n",sumByte);
        
        AVPacket *packet = NULL;
        packet = (AVPacket *)malloc(sizeof(AVPacket));
        
        if (packet == NULL)
        {
            return;
        }
        packet->data = (uint8_t *)malloc(sumByte);
        packet->size = sumByte;
        
        //debug_info("NewByteArray memcpy \n");
        if (!fHaveWrittenFirstFrame) {
            // If we have NAL units encoded in "sprop parameter strings", prepend these to the file:
            for (unsigned j = 0; j < 3; ++j) {
                for (unsigned i = 0; i < numSPropRecords[j]; ++i) {
                    
                    memcpy(packet->data + writeByte, start_code, 4);
                    
                    writeByte += 4;
                    
                    memcpy(packet->data + writeByte, sPropRecords[j][i].sPropBytes,sPropRecords[j][i].sPropLength);
                    
                    writeByte += sPropRecords[j][i].sPropLength;
#ifdef FILE_SAVE_ENB
                    addData(start_code, 4);
                    addData(sPropRecords[j][i].sPropBytes, sPropRecords[j][i].sPropLength);
#endif /* FILE_SAVE_ENB */
                }
                delete[] sPropRecords[j];
            }
            fHaveWrittenFirstFrame = True; // for next time
        }
        
        memcpy(packet->data + writeByte, start_code, 4);
        
        // Write  front data to per frame
        
        
#ifdef FILE_SAVE_ENB
        addData(start_code, 4);
#endif /* FILE_SAVE_ENB */
        writeByte += 4;
        memcpy(packet->data + writeByte, fBuffer, frameSize);
#ifdef FILE_SAVE_ENB
        addData(fBuffer, frameSize);
#endif /* FILE_SAVE_ENB */
        writeByte += frameSize;
        debug_info("afterGettingFrame writeByte = %d\n",writeByte);
        video_packet_queue_push_head(packet);
    }
    
#ifdef CHECK_IDRFRM_ENB
SKIP_FRAME:
#endif /* CHECK_IDRFRM_ENB */
    // Then continue, to request the next frame of data:
    continuePlaying();
}

void HEVCSink::addData(unsigned char const* data, unsigned dataSize) {
#ifdef FILE_SAVE_ENB
    //fwrite(data, 1, dataSize, fOutFid);
    write(fd, data, dataSize);
#endif /* FILE_SAVE_ENB */
}

Boolean HEVCSink::continuePlaying() {
    // sanity check (should not happen)
    if (fSource == NULL)
        return False;
    
    /* Request the next frame of data from our input source.
     "afterGettingFrame()" will get called later, when it arrives: */
    fSource->getNextFrame(fBuffer, HEVC_SINK_RECEIVE_BUFFER_SIZE,
                          afterGettingFrame, this, onSourceClosure, this);
    return True;
}

//static void* main_setup_rtsp(void* rv) {
//    // Begin by setting up our usage environment:
//    char* url = (char*) rv;
//    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
//    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);
//    
//    // We need at least one "rtsp://" URL argument:
//    //open user rtsp URL
//    openURL(*env, "HEVCDecoderTest", url);
//    OutPacketBuffer::maxSize = 600000;
//    
//    eventLoopWatchVariable = 0;
//    
//    // All subsequent activity takes place within the event loop:
//    env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
//    if ((NULL != env)) {
//        env->reclaim();
//    }
//    if ((NULL != scheduler)) {
//        delete scheduler;
//        scheduler = NULL;
//    }
//    return NULL;
//}

void rtspMainTask(uint8_t *rv, playStatus p)
{
    // Begin by setting up our usage environment:
    pStatus = p;
    char* url = (char*) rv;
    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);
    
    //open user rtsp URL
    openURL(*env, "HEVCDecoderTest", url);
    OutPacketBuffer::maxSize = 600000;
    
    eventLoopWatchVariable = 0;
    // All subsequent activity takes place within the event loop:
    env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
    if ((NULL != env)) {
        env->reclaim();
    }
    if ((NULL != scheduler)) {
        delete scheduler;
        scheduler = NULL;
    }
    return;
}


void rtspMainTaskStop()
{
    if (rtspClient == NULL) {
        return;
    }
    StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias
    if (scs.session != NULL) {
        rtspClient->sendTeardownCommand(*scs.session, NULL);
    }
    rtspClient->resetTCPSockets();
    eventLoopWatchVariable = 1;
}

//=======================native function==============
//static const char* const kClass_RenderActivity = "com/ipc/mediacodec/VideoDecoder";
//static const char* const kClass_RenderActivity =
//"com/android/mediacodec/VideoDecoder";

//struct fields_t {
//	jmethodID post_event;
//	jmethodID exit_event;
//};
//static fields_t fields;
//jobject mObject;
static bool g_bAttatedT;
//static JavaVM *g_JavaVM;

//static pthread_t threadId = 0;
//void notify(int msg, const jbyteArray cbb, long presentationTime) {
//	//ALOGD("JNIIpcClientListener::notify");
//	JNIEnv *env = GetEnv();
//	if (cbb != NULL) {
//		jbyteArray callbackBuffer = (jbyteArray) env->NewLocalRef(cbb);
//		env->CallVoidMethod(mObject, fields.post_event, msg, callbackBuffer,
//				presentationTime);
//		env->DeleteLocalRef(callbackBuffer);
//	} else {
//		env->CallVoidMethod(mObject, fields.post_event, msg, NULL);
//	}
//	if (env->ExceptionCheck()) {
//		//ALOGW("An exception occurred while notifying an event.");
//		env->ExceptionClear();
//	}
//}

//void eixt_Notify() {
//	//ALOGD("JNIIpcClientListener::eixt_Notify");
//	JNIEnv *env = GetEnv();
//	env->CallVoidMethod(mObject, fields.exit_event);
//	if (env->ExceptionCheck()) {
//		//ALOGW("An exception occurred while notifying an exit event.");
//		env->ExceptionClear();
//	}
//}

//void native_init(JNIEnv *env, jobject thiz, jstring eturl) {
//	const char *tmpUrl = NULL;
//	//pthread_attr_t attr;
//	//struct sched_param param;
//	//pthread_attr_setschedpolicy(&attr, SCHED_RR);
//	//param.sched_priority = 99;
//	//pthread_attr_setschedparam(&attr, &param);
//	//pthread_attr_init (&attr);
//	//pthread_attr_setscope (&attr, PTHREAD_SCOPE_SYSTEM);
//	//pthread_attr_setdetachstate (&attr, PTHREAD_CREATE_DETACHED);
//	init();
//	//ALOGE("GUO native enter\n");
//
//	tmpUrl = env->GetStringUTFChars(eturl, 0);
//	if (tmpUrl == NULL) { // Out of memory
//		//ALOGE("GUO URL is error [Out of memory]\n");
//		return;
//	}
//	if ((pthread_create(&threadId, NULL, main_setup_rtsp, (void*) tmpUrl) < 0)) {
//		//ALOGE("GUO create thread failed\n");
//		return;
//	}
//	//ALOGE("GUO rtspClient end\n");
//	mObject = env->NewGlobalRef(thiz);
//	//ALOGE("GUO NewGlobalRef\n");
//	jclass string_class = env->GetObjectClass(thiz);
//	//ALOGE("GUO GetObjectClass\n");
//	fields.post_event = env->GetMethodID(string_class, "postDataFromNative",
//			"(I[BJ)V");
//	fields.exit_event = env->GetMethodID(string_class, "exitNotify", "()V");
//	//pthread_attr_destroy(&attr);
//	//ALOGE("GUO GetMethodID\n");
//}
//
//void native_deinit(JNIEnv *env, jobject thiz) {
//	//ALOGE("GUO native deinit enter[%d][%d] \n",
//	//eventLoopWatchVariable, threadId);
//	if ((threadId != 0)) {
//		eventLoopWatchVariable = 1;
//		pthread_join(threadId, NULL);
//	}
//
//	//ALOGE("GUO native deinit out[%d][%d] \n",
//	//eventLoopWatchVariable, threadId);
//}
//
//static JNINativeMethod gMethods[] = {
//
//{ "native_init", "(Ljava/lang/String;)V", (void *) native_init }, {
//		"native_deinit", "()V", (void *) native_deinit }, };
//
//int register_RenderActivity(JNIEnv *env) {
//	jclass clazz = (env)->FindClass(kClass_RenderActivity);
//	return (env)->RegisterNatives(clazz, gMethods, 2);
//}
//
//jint JNI_OnLoad(JavaVM* vm, void* /* reserved */) {
//	JNIEnv* env = NULL;
//	g_JavaVM = vm;
//	jint result = -1;
//
//	if (vm->GetEnv((void**) &env, JNI_VERSION_1_4) != JNI_OK || env == NULL) {
//		//ALOGE("ERROR: GetEnv failed\n");
//		goto bail;
//	}
//
//	if (register_RenderActivity(env) < 0) {
//		//ALOGE("ERROR: register_IPCamera failed");
//		goto bail;
//	}
//	/* success -- return valid version number */
//	result = JNI_VERSION_1_4;
//
//	bail: return result;
//}

void init() {
    g_bAttatedT = false;
    //g_JavaVM = android::AndroidRuntime::getJavaVM();
}

//static JNIEnv *GetEnv() {
//	int status;
//	JNIEnv *envnow = NULL;
//	status = g_JavaVM->GetEnv((void **) &envnow, JNI_VERSION_1_4);
//	if (status < 0) {
//		status = g_JavaVM->AttachCurrentThread(&envnow, NULL);
//		if (status < 0) {
//			return NULL;
//		}
//		g_bAttatedT = true;
//	}
//	return envnow;
//}

//static void DetachCurrent() {
//    if (g_bAttatedT) {
//        g_bAttatedT = false;
//    }
//}

