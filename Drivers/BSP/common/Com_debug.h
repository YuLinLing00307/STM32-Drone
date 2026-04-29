#ifndef __COM_DEBUG_H
#define __COM_DEBUG_H

#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

// 日志打印输出非常耗用时间,在实机上需要关闭
// 可以设置一个开关，使用宏定义
#define DEBUG_LOG_ENABLE 1

// 可以采用宏定义，只打印文件名而不打印路径
// strrchr用于对字符串从后往前查找内容,返回的是查找到的位置的指针
#define __FILE_NAME__ (strrchr(__FILE__,'/')?strrchr(__FILE__,'/'):__FILE__)

#ifdef DEBUG_LOG_ENABLE
// 替换调试输出函数
#define debug_printf(format,...) printf("[%s:%d]" format,__FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else // 若不开启的话，使用一个空函数，避免报错
#define debug_printf(format,...) 
#endif

#endif // 
