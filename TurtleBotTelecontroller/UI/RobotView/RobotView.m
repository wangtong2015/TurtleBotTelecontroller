//
//  RobotView.m
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/8.
//  Copyright © 2018年 WangTong. All rights reserved.
//

#import "RobotView.h"
#import "ASValueTrackingSlider.h"
#import "RockerView.h"
#import "NetworkManager.h"
@interface RobotView()
// UI
@property (strong, nonatomic) UIView * containerView;
@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (weak, nonatomic) IBOutlet UITextView *textView;
@property (weak, nonatomic) IBOutlet ASValueTrackingSlider *speedView;
@property (weak, nonatomic) IBOutlet RockerView *rockerView;
@property (weak, nonatomic) IBOutlet UILabel *labelView;
@property (weak, nonatomic) IBOutlet UISwitch *switchView;



@property (weak, nonatomic) IBOutlet UITextField *imageDownloadTextField;
@property (weak, nonatomic) IBOutlet UITextField *statusDownloadTextField;
@property (weak, nonatomic) IBOutlet UITextField *statusUploadTextField;
@property (weak, nonatomic) IBOutlet UITextField *controlDownloadTextField;
@property (weak, nonatomic) IBOutlet UITextField *controlUploadTextField;




// Action
- (IBAction)switchAction:(id)sender;

// Network
@property (strong, nonatomic) NetworkManager * networkManager;

// Data
@property (strong, nonatomic) NSDictionary * robotDict;
@property (strong, nonatomic) NSDictionary * controlDict;
@end


@implementation RobotView
- (instancetype) initWithCoder:(NSCoder *)aDecoder{
    if (self = [super initWithCoder:aDecoder]) {
        _containerView = [[[UINib nibWithNibName:@"RobotView" bundle:nil] instantiateWithOwner:self options:nil] firstObject];
        [self addSubview:self.containerView];
        self.switchView.userInteractionEnabled = YES;
        [self addGestureRecognizerToView:self.imageView];
        
        // 网络
        _networkManager = [NetworkManager sharedInstance];
        self.networkManager.ifLoop = self.switchView.on;
        
        self.imageDownloadTextField.text = self.networkManager.ImageRoot;
        self.statusDownloadTextField.text = self.networkManager.RobotDownload;
        self.statusUploadTextField.text = self.networkManager.RobotUpload;
        self.controlDownloadTextField.text = self.networkManager.ControlDownload;
        self.controlUploadTextField.text = self.networkManager.ControlUpload;
    }
    return self;
}

- (void)awakeFromNib{
    [super awakeFromNib];
    CGFloat wRatio = self.bounds.size.width / self.containerView.frame.size.width;
    CGFloat hRatio = self.bounds.size.height / self.containerView.frame.size.height;
    
    self.containerView.frame = CGRectMake(0, 0, self.bounds.size.width, self.bounds.size.height);
    for (UIView * subView in self.containerView.subviews) {
        subView.frame = CGRectMake(subView.frame.origin.x * wRatio, subView.frame.origin.y * hRatio, subView.bounds.size.width * wRatio, subView.bounds.size.height * hRatio);
    }
    
    __weak typeof(self) weakSelf = self;
    self.networkManager.ImageUpdate = ^(UIImage *newImage) {
        NSLog(@"newImage");
        dispatch_async(dispatch_get_main_queue(), ^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            strongSelf.imageView.image = newImage;
        });
    };
    
    self.networkManager.robotUpdate = ^(NSDictionary *newRobotDict) {
        NSLog(@"newRobotDict");
        __strong typeof(weakSelf) strongSelf = weakSelf;
        strongSelf.robotDict = newRobotDict;
    };
    
    self.rockerView.changeBlock = ^(CGFloat r, CGFloat theta, BOOL important) {
        __strong typeof(weakSelf) strongSelf = weakSelf;
        NSString * rValue = [NSString stringWithFormat:@"%.3f",r * self.speedView.value];
        NSString * thetaValue = [NSString stringWithFormat:@"%.3f",theta];
        NSDictionary * newControlDict = @{rKey:rValue, thetaKey:thetaValue};
        strongSelf.controlDict = newControlDict;
        [self.networkManager controlWithDict:newControlDict important:important];
    };
    
    [self.rockerView changeParameter:YES];
}



/*
// Only override drawRect: if you perform custom drawing.
// An empty implementation adversely affects performance during animation.
- (void)drawRect:(CGRect)rect {
    // Drawing code
}
*/

#pragma mark -- UI Action
- (IBAction)switchAction:(id)sender {
    self.networkManager.ifLoop = self.switchView.on;
}

#pragma mark -- Set
- (void)setControlDict:(NSDictionary *)controlDict{
    _controlDict = controlDict;
    [self resetText];
}

- (void)setRobotDict:(NSDictionary *)robotDict{
    _robotDict = robotDict;
    [self resetText];
}

#pragma mark -- Private
- (void)resetText{
    NSString * text = @"";
    for (NSString * key in self.controlDict.allKeys) {
        NSString * value = self.controlDict[key];
        text = [text stringByAppendingFormat:@"%@ = %@\n",key, value];
    }
    for (NSString * key in self.robotDict.allKeys) {
        NSString * value = self.robotDict[key];
        text = [text stringByAppendingFormat:@"%@ = %@\n",key, value];
    }
    dispatch_async(dispatch_get_main_queue(), ^{
        self.textView.text = text;
    });
}


#pragma mark -- 图片
// 添加所有的手势
- (void) addGestureRecognizerToView:(UIView *)view
{
    // 旋转手势
    UIRotationGestureRecognizer *rotationGestureRecognizer = [[UIRotationGestureRecognizer alloc] initWithTarget:self action:@selector(rotateView:)];
    [view addGestureRecognizer:rotationGestureRecognizer];
    
    // 缩放手势
    UIPinchGestureRecognizer *pinchGestureRecognizer = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(pinchView:)];
    [view addGestureRecognizer:pinchGestureRecognizer];
    
    // 移动手势
    UIPanGestureRecognizer *panGestureRecognizer = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(panView:)];
    [view addGestureRecognizer:panGestureRecognizer];
}

// 处理旋转手势
- (void) rotateView:(UIRotationGestureRecognizer *)rotationGestureRecognizer
{
    UIView *view = rotationGestureRecognizer.view;
    if (rotationGestureRecognizer.state == UIGestureRecognizerStateBegan || rotationGestureRecognizer.state == UIGestureRecognizerStateChanged) {
        view.transform = CGAffineTransformRotate(view.transform, rotationGestureRecognizer.rotation);
        [rotationGestureRecognizer setRotation:0];
    }
}

// 处理缩放手势
- (void) pinchView:(UIPinchGestureRecognizer *)pinchGestureRecognizer
{
    UIView *view = pinchGestureRecognizer.view;
    if (pinchGestureRecognizer.state == UIGestureRecognizerStateBegan || pinchGestureRecognizer.state == UIGestureRecognizerStateChanged) {
        view.transform = CGAffineTransformScale(view.transform, pinchGestureRecognizer.scale, pinchGestureRecognizer.scale);
        pinchGestureRecognizer.scale = 1;
    }
}

// 处理拖拉手势
- (void) panView:(UIPanGestureRecognizer *)panGestureRecognizer
{
    UIView *view = panGestureRecognizer.view;
    if (panGestureRecognizer.state == UIGestureRecognizerStateBegan || panGestureRecognizer.state == UIGestureRecognizerStateChanged) {
        CGPoint translation = [panGestureRecognizer translationInView:view.superview];
        [view setCenter:(CGPoint){view.center.x + translation.x, view.center.y + translation.y}];
        [panGestureRecognizer setTranslation:CGPointZero inView:view.superview];
    }
}
- (IBAction)textFieldEnd:(UITextField *)sender {
    [sender resignFirstResponder];
    self.networkManager.ImageRoot = self.imageDownloadTextField.text;
    self.networkManager.RobotUpload = self.statusUploadTextField.text;
    self.networkManager.RobotDownload = self.statusDownloadTextField.text;
    self.networkManager.ControlUpload = self.controlUploadTextField.text;
    self.networkManager.ControlDownload = self.controlDownloadTextField.text;
}

@end
