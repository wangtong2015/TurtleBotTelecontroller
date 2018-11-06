//
//  GeometryUtil.c
//  TurtleBotTelecontroller
//
//  Created by WangTong on 2018/8/8.
//  Copyright © 2018年 WangTong. All rights reserved.
//

#include "GeometryUtil.h"
#include "math.h"

CGFloat Clamp(CGFloat value){
    if (value >= 1) {
        return 1;
    }else if (value <= 0){
        return 0;
    }
    return value;
}

CGRect CGRectMulRatio(CGRect rect, CGFloat ratio){
    return CGRectMake(rect.origin.x * ratio, rect.origin.y * ratio, rect.size.width * ratio, rect.size.height * ratio);
}

CGRect CGRectDivRatio(CGRect rect, CGFloat ratio){
    return CGRectMake(rect.origin.x / ratio, rect.origin.y / ratio, rect.size.width / ratio, rect.size.height / ratio);
}

CGFloat CGRectGetRatio(CGRect huge, CGRect small, bool alignLongEdge){
    CGFloat ratio;
    if (alignLongEdge) {
        ratio = WTMAX(huge.size.width / small.size.width , huge.size.height / small.size.height);
    }else{
        ratio = WTMIN(huge.size.width / small.size.width , huge.size.height / small.size.height);
    }
    return ratio;
}

CGRect CGRectNorm(CGRect rect, CGSize size){
    return CGRectMake(rect.origin.x / size.width, rect.origin.y / size.height, rect.size.width / size.width, rect.size.height / size.height);
}

CGRect CGRectDeNorm(CGRect rect, CGSize size){
    return CGRectMake(rect.origin.x * size.width, rect.origin.y * size.height, rect.size.width * size.width, rect.size.height * size.height);
}

CGPoint CGRectGetCenter(CGRect rect){
    return CGPointMake(CGRectGetMidX(rect), CGRectGetMidY(rect));
}

CGRect CGRectFromCenter(CGPoint center, CGSize size){
    return CGRectMake(center.x + size.width/2, center.y + size.height/2, size.width, size.height);
}

CGFloat CGPointGetDis(CGPoint p1, CGPoint p2){
    return sqrt(WTPOW2(p1.x - p2.x) + WTPOW2(p1.y - p2.y));
}













