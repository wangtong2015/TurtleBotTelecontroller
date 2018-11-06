//
//  SynthesizeSingletonARC.h
//  Netdisk_Mac
//
//  Created by wuxiaoyue on 13-3-4.
//  Copyright (c) 2013年 wuxiaoyue. All rights reserved.
//
#define SYNTHESIZE_SINGLETON_DEF(className) \
\
+ (className *)sharedInstance;\
+(instancetype) alloc __attribute__((unavailable("call sharedInstance instead")));\
+(instancetype) new __attribute__((unavailable("call sharedInstance instead")));\
-(instancetype) copy __attribute__((unavailable("call sharedInstance instead")));\
-(instancetype) mutableCopy __attribute__((unavailable("call sharedInstance instead")));\


#define SYNTHESIZE_SINGLETON_IMP(className) \
\
+ (className *)sharedInstance { \
static className *_shared##className = nil; \
static dispatch_once_t onceToken; \
dispatch_once(&onceToken, ^{ \
_shared##className = [[super alloc] init]; \
}); \
return _shared##className; \
}
