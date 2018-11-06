//
//  ViewController.m
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/5.
//  Copyright © 2018年 WangTong. All rights reserved.
//

#import "ViewController.h"
#define Width [UIScreen mainScreen].bounds.size.width
#define Height [UIScreen mainScreen].bounds.size.height

@interface ViewController ()

@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    NSLog(@"%f,%f", Width, Height);
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
