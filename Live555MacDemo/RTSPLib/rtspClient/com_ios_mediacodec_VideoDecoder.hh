//
//  com_ios_mediacodec_VideoDecoder.hpp
//  RTSPBasedPlayer
//
//  Created by thundersoft on 16/5/25.
//  Copyright © 2016年 thundersoft. All rights reserved.
//

#ifndef com_ios_mediacodec_VideoDecoder_hpp
#define com_ios_mediacodec_VideoDecoder_hpp

#include <stdio.h>
#include <stdlib.h>


#ifdef DEBUG
#define debug_info(fmt, args...) \
fprintf(stdout, fmt, ##args)
#else
#define debug_info(fmt, args...)
#endif /* DEBUG */


#ifdef __cplusplus
extern "C" {
#endif
    
typedef enum
{
    RTSP_SERVER_CONNECT_ERROR,
    RTSP_SERVER_CONNECTED,
    RTSP_PLAYER_PLAY_ERROR,
    RTSP_PLAYER_PLAY,
    RTSP_PLAYER_PLAY_STOPPED
} play_status;
    
 typedef int (*playStatus)(int isPlaying);
 extern void rtspMainTask(uint8_t *rv, playStatus p);
 extern void rtspMainTaskStop();
    
#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* com_ios_mediacodec_VideoDecoder_hpp */


