//
//  NetworkManager.h
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/8.
//  Copyright © 2018年 WangTong. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import "SynthesizeSingletonARC.h"

static NSString * const ImageRoot_S = @"http://wangtong15.com:10000/static/images/";
static NSString * const RobotDownload_S = @"http://wangtong15.com:10004/static/data/robot.json";
static NSString * const ControlDownload_S = @"http://wangtong15.com:10005/static/data/control.json";
static NSString * const ControlUpload_S = @"http://wangtong15.com:10002/BroadCast/?control";
static NSString * const RobotUpload_S = @"http://wangtong15.com:10003/BroadCast/?robot";

#pragma mark -- Dict KEY
static NSString * const uploadTimeKey = @"uploadTime";
static NSString * const rKey = @"r";
static NSString * const thetaKey = @"theta"; // 角度
static NSString * const imageNameKey = @"imageName";

@interface NetworkManager : NSObject

@property (nonatomic, assign) BOOL ifLoop; // YES表示下载信息，NO表示不下载信息

@property (nonatomic, copy) void (^robotUpdate)(NSDictionary * newRobotDict);
@property (nonatomic, copy) void (^ImageUpdate)(UIImage * newImage);

// 网址
@property (nonatomic, strong) NSString * ImageRoot;
@property (nonatomic, strong) NSString * RobotDownload;
@property (nonatomic, strong) NSString * ControlDownload;
@property (nonatomic, strong) NSString * ControlUpload;
@property (nonatomic, strong) NSString * RobotUpload;



SYNTHESIZE_SINGLETON_DEF(NetworkManager)
-(void)controlWithDict:(NSDictionary*)controlDict important:(BOOL)important;
@end
