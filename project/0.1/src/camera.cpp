/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/camera.h"

namespace myslam
{

Camera::Camera()
{
}

/* 下面主要介绍了各类坐标系之间的变换函数
 *
 * p_w:世界坐标系下P: (X,Y,Z)
 * p_c:相机坐标系下的坐标（x,y,z）
 * p_p:像素坐标（u,v）
 * T_c_w:相机到世界的变换矩阵
 */
Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )//世界坐标系下P:（X,Y,Z）,T:变换矩阵
{
    return T_c_w*p_w;//相机坐标（x,y,z）=T×P
}

Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

Vector2d Camera::camera2pixel ( const Vector3d& p_c )//相机到像素
{
    return Vector2d (
        fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
        fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
    );
}

Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )//像素到相机
{
    return Vector3d (
        ( p_p ( 0,0 )-cx_ ) *depth/fx_,
        ( p_p ( 1,0 )-cy_ ) *depth/fy_,
        depth
    );
}

Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )//世界到像素
{
    return camera2pixel ( world2camera ( p_w, T_c_w ) );
}

Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


}
