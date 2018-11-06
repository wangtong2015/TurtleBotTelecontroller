//
//  RockerView.m
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/6.
//  Copyright © 2018年 WangTong. All rights reserved.
//

#import "RockerView.h"
#import "GeometryUtil.h"
@interface RockerView()
@property (strong, nonatomic) UIView * containerView;
@property (weak, nonatomic) IBOutlet UIImageView * headView;
@property (weak, nonatomic) IBOutlet UIImageView * baseView;

@end

@implementation RockerView

+ (RockerView *)RockerViewWithFrame:(CGRect)frame
{
    RockerView *rocker = [[[NSBundle mainBundle] loadNibNamed:@"RockerView" owner:nil options:nil] lastObject];
    rocker.frame = frame;
    return rocker;
}

- (instancetype)initWithCoder:(NSCoder *)aDecoder{
    if (self = [super initWithCoder:aDecoder]) {
        _containerView = [[[UINib nibWithNibName:@"RockerView" bundle:nil] instantiateWithOwner:self options:nil] firstObject];
        [self addSubview:self.containerView];
        self.containerView.frame = CGRectMake(0, 0, self.bounds.size.width, self.bounds.size.height);
    }
    return self;
}
- (void)awakeFromNib
{
    [super awakeFromNib];
    // 自适应
    CGFloat scale = CGRectGetRatio(self.containerView.bounds, self.baseView.bounds, NO);
    self.baseView.bounds = CGRectMulRatio(self.baseView.bounds, scale);
    self.baseView.center = CGRectGetCenter(self.containerView.bounds);
    self.headView.bounds = CGRectMulRatio(self.headView.bounds, scale);
    self.headView.center = self.baseView.center;
    

    self.r = 0;
    self.theta = 0;
    [self changeParameter:YES];
}

// 只要监听的方法一改变，就会调用观察者的这个方法，通知你有新值
-(void)observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context{

}


- (void)touchesBegan:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event
{
    //    NSLog(@"touchesBegan");
    
    UITouch *touchCenter = [touches anyObject];
    
    CGPoint point1 = [touchCenter locationInView:self];
    
    [self point:point1 inCircleRect:self.baseView.frame];
    [self changeParameter:YES];
}

- (void)touchesMoved:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event
{
    UITouch *touchCenter = [touches anyObject];
    
    CGPoint point1 = [touchCenter locationInView:self];
    
    [self point:point1 inCircleRect:self.baseView.frame];
    [self changeParameter:NO];
}

- (void)touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event
{
    NSLog(@"Touch End");
    self.headView.center = self.baseView.center;
    self.r = 0;
    self.theta = 0;
    [self changeParameter:YES];
}

//圆心到点的距离 > 半径
- (void)point:(CGPoint)point inCircleRect:(CGRect)rect {
    
    CGFloat r = CGRectGetWidth(rect)/2.0;   //圆的半径
    
    CGPoint center = CGRectGetCenter(rect);
    
    double a = point.x - center.x;    // a            /|
    double b = - (point.y - center.y);    // b         c / |b
    double c = sqrt(a * a + b * b);                 // c          /__|
    double cosθ = a / c;
    double sinθ = b / c;
    
    if (cosθ < sinθ) {
        if (b > 0) {
            self.theta = acos(cosθ);
        }else{
            self.theta = - acos(cosθ);
        }
    }else{
        if (a > 0) {
            self.theta = asin(sinθ);
        }else{
            if (b > 0) {
                self.theta = M_PI - asin(sinθ);
            }else{
                self.theta = - M_PI - asin(sinθ);
            }
        }
    }
    
    if (c <= r) {   //点击的位置在圆内
        self.headView.center = point;
        self.r = c;
    } else {
        //圆参数方程  x = m + r * cosθ, y = n + r * sinθ (m,n)为圆心
        self.headView.center = CGPointMake(center.x + r * cosθ ,  // x = m + r * cosθ
                                                center.y - r * sinθ);  // y = n + r * sinθ
        self.r = r;
    }

    // 边长归一化
    self.r /= r;
    self.theta *= 180.0 / M_PI;
}

- (void)changeParameter:(BOOL)important{
    NSLog(@"r:%.3f ---  θ:%.3f", self.r, self.theta);
    if (self.changeBlock) {
        self.changeBlock(self.r, self.theta, important);
    }
}

@end
