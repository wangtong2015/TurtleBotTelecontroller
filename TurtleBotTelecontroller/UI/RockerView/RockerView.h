//
//  RockerView.h
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/6.
//  Copyright © 2018年 WangTong. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface RockerView : UIView
@property (nonatomic, assign) CGFloat r;
@property (nonatomic, assign) CGFloat theta;
@property (nonatomic, copy) void (^changeBlock)(CGFloat r, CGFloat theta, BOOL important);

+ (RockerView *)RockerViewWithFrame:(CGRect)frame;

- (void)changeParameter:(BOOL)important;
@end
