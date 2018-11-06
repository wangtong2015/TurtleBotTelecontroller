//
//  NetworkManager.m
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/8.
//  Copyright © 2018年 WangTong. All rights reserved.
//


#import "NetworkManager.h"
#import <UIKit/UIKit.h>




@interface NetworkManager()
@property (nonatomic, strong) NSTimer * timer;

@property (nonatomic, strong) NSString * latestUploadTime;
@property (nonatomic, strong) NSString * latestImageName;

// Json Download
@property (nonatomic, strong) NSURL * robotURL;

// Json Upload
@property (nonatomic, strong) NSURL * controlURL;
@property (nonatomic, strong) NSURLRequest * controlRequest;
@property (nonatomic, strong) NSURLSession * controlSession;
@property (nonatomic, strong) NSURLSessionDataTask * controlTask;
@property (atomic, assign) BOOL uploadComplete;

@property (nonatomic, strong) NSThread * thread;

@end


@implementation NetworkManager
SYNTHESIZE_SINGLETON_IMP(NetworkManager)

- (instancetype)init{
    if (self = [super init]) {
        // Network init
        self.ifLoop = NO;
        self.ImageRoot = ImageRoot_S;
        self.RobotDownload = RobotDownload_S;
        self.RobotUpload = RobotUpload_S;
        self.ControlDownload = ControlDownload_S;
        self.ControlUpload = ControlUpload_S;
        
        
        _thread = nil;
        _uploadComplete = YES;
        _latestImageName = nil;
    }
    return self;
}


- (void)setIfLoop:(BOOL)ifLoop{
    _ifLoop = ifLoop;
    if(!ifLoop && self.thread){
        [self.thread cancel];
        self.thread = nil;
    }else if (ifLoop && !self.thread){
        self.thread = [[NSThread alloc] initWithTarget:self selector:@selector(loop) object:nil];
        [self.thread setName:@"TurtleNetwork"];
        [self.thread start];
    }
}

- (void)loop{
    while (1) {
        @autoreleasepool{
            // Robot
            if ([[NSThread currentThread] isCancelled]) {
                [NSThread exit];
                return;
            }
            NSError *error = nil;
            NSData * robotData = [NSData dataWithContentsOfURL:self.robotURL options:NSDataReadingMappedIfSafe|NSDataReadingUncached error:&error];
            if (error) {
                NSLog(@"获取机器人状态失败:%@",error.localizedDescription);
                continue;
            }
            NSDictionary * robotDict = [NSJSONSerialization JSONObjectWithData:robotData options:NSJSONReadingMutableLeaves error:nil];
            NSString * imageName = robotDict[imageNameKey];
            if ([imageName isEqualToString:self.latestImageName]) {
                continue;
            }
            self.latestImageName = imageName;
            if (self.robotUpdate) {
                self.robotUpdate(robotDict);
            }
            NSString * imagePath = [_ImageRoot stringByAppendingPathComponent:imageName];
            NSURL * imageURL = [NSURL URLWithString:imagePath];
            error = nil;
            NSData * imageData = [NSData dataWithContentsOfURL:imageURL options:NSDataReadingMappedIfSafe|NSDataReadingUncached error:&error];
            if (error) {
                NSLog(@"获取机器人图片失败:%@",error.localizedDescription);
                continue;
            }
            UIImage * image = [UIImage imageWithData:imageData];
            if (self.ImageUpdate) {
                self.ImageUpdate(image);
            }
        }
    }
}

-(void)controlWithDict:(NSDictionary*)controlDict important:(BOOL)important{
    dispatch_async(dispatch_get_global_queue(0, 0), ^{
        if (!self.uploadComplete && !important) {
            return;
        }
        self.uploadComplete = NO;
        CFAbsoluteTime start = CFAbsoluteTimeGetCurrent();
        NSString * text = [NSString stringWithFormat:@"%@",self.ControlUpload];
        for (NSString * key in controlDict.allKeys) {
            NSString * value = controlDict[key];
            NSString * appendText = [NSString stringWithFormat:@"&%@=%@",key,value];
            text = [text stringByAppendingString:appendText];
        }
        NSURL * url = [NSURL URLWithString:text];
        NSURLRequest *request = [[NSURLRequest alloc] initWithURL:url];
        NSURLSession *session = [NSURLSession sharedSession];
        NSURLSessionDataTask *task = [session dataTaskWithRequest:request completionHandler:^(NSData *data, NSURLResponse *response, NSError *error){
            if (error) {
                NSLog(@"control error : %@", error.localizedDescription);
            }
            NSLog(@"control:%f", CFAbsoluteTimeGetCurrent() - start);
            self.uploadComplete = YES;
        }];
        [task resume];
    });
}


#pragma mark -- set
- (void)setRobotDownload:(NSString *)RobotDownload{
    _RobotDownload = RobotDownload;
    _robotURL = [NSURL URLWithString:_RobotDownload];
}

- (void)setControlDownload:(NSString *)ControlDownload{
    _ControlDownload = ControlDownload;
    _controlURL = [NSURL URLWithString:_ControlDownload];
}
@end
