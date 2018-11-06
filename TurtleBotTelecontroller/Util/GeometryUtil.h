//
//  GeometryUtil.h
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/8.
//  Copyright © 2018年 WangTong. All rights reserved.
//

#ifndef GeometryUtil_h
#define GeometryUtil_h

#include <stdio.h>
#include <CoreGraphics/CGGeometry.h>

#define PI 3.1415926535
#define WTMAX(x,y) ((x) > (y) ? (x) : (y))
#define WTMIN(x,y) ((x) < (y) ? (x) : (y))
#define WTPOW2(x) ((x)*(x))
#define WTAngleToRadian(x) ((x) * PI / 360.0)
#define WTRadianToAngle(x) ((x) * 360.0 / PI)

CGFloat Clamp(CGFloat value); // 0-1

CGRect CGRectMulRatio(CGRect rect, CGFloat ratio);

CGRect CGRectDivRatio(CGRect rect, CGFloat ratio);

CGFloat CGRectGetRatio(CGRect huge, CGRect small, bool alignLongEdge);

CGRect CGRectNorm(CGRect rect, CGSize size);

CGRect CGRectDeNorm(CGRect rect, CGSize size);

CGPoint CGRectGetCenter(CGRect rect);

CGRect CGRectFromCenter(CGPoint center, CGSize size);

CGFloat CGPointGetDis(CGPoint p1, CGPoint p2);

#endif /* GeometryUtil_h */
