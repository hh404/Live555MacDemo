//
//  main.m
//  Live555MacDemo
//
//  Created by huangjianwu on 2017/1/18.
//  Copyright © 2017年 huangjianwu. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "playCommon.hh"

int main(int argc, const char * argv[]) {
    @autoreleasepool {
        // insert code here...
        NSLog(@"Hello, World!");
        //openRTSP(argc,argv);
        char *p[] = {"openRTSP","-d","20","rtsp://192.168.123.244:8554/mtvh.264"};
        openRTSP(4, p);
    }
    return 0;
}
