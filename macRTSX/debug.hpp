//                       ____ _____ ______  __
//  _ __ ___   __ _  ___|  _ \_   _/ ___\ \/ /
// | '_ ` _ \ / _` |/ __| |_) || | \___ \\  /
// | | | | | | (_| | (__|  _ < | |  ___) /  \
// |_| |_| |_|\__,_|\___|_| \_\|_| |____/_/\_\

#pragma once

#include <IOKit/IOLib.h>

class Logger
{
	const char *name_;

public:
	Logger(const char *);
	
	template <typename T, typename... Args>
	void normal(T& msg, Args... args)
	{
		IOLog("%s:   ", name_);
		IOLog(msg, args...);
	}
	
	template <typename T, typename... Args>
	void debug(T& msg, Args... args)
	{
		IOLog("%s   (Debug):", name_);
		IOLog(msg, args...);
	}
	
	/*
	 * I would really love to have a backtrace here.
	 * If there is some kind of built-in I don't know about...
	 */
	template <typename T, typename... Args>
	void error(T& msg, Args... args)
	{
		IOLog("%s   (Error):", name_);
		IOLog(msg, args...);
	}
	
	template <typename T, typename... Args>
	void development(T& msg, Args... args)
	{
		IOLog("%s   (Development):", name_);
		IOLog(msg, args...);
	}

};
