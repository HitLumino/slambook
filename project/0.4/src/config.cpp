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

#include "myslam/config.h"

namespace myslam
{

void Config::setParameterFile( const std::string& filename )//公有静态方法，静态变量是`config_`，只有这个*内部函数*能实例化`config()`
{
    if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);//内部调用私有构造函数`Config()`,返回对象“config_”的实例，这个实例也在内部定义好了，即静态成员变量  `private static std::shared_ptr<Config> config_;`
     //这里体现了什么叫唯一性：如果对象为空，就初始化新的对象；若不为空，即不新建。则始终保持一个对象（只有这个静态方法）
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;

}
