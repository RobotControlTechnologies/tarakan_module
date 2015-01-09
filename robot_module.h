/* 
 * File:   robots.h
 * Author: m79lol
 *
 * Created on 2 Июнь 2014 г., 17:39
 */

#ifndef ROBOTS_H
#define	ROBOTS_H

class Robot {
    protected:
        Robot() {}
    public: 
        virtual regval executeFunction(regval command_index, regval *args) = 0;
        virtual ~Robot() {}
};

class RobotModule {
    protected:
        RobotModule() {}
    public: 
        virtual int init() = 0;
        virtual FunctionData* checkAviableFunction(const char *function_name) = 0;
        virtual Robot* robotRequire() = 0;
        virtual void robotFree(Robot *robot) = 0;
        virtual void final() = 0;
		virtual void destroy() = 0;
        virtual ~RobotModule() {}
};
typedef RobotModule* (*getRobotModuleObject_t)();

#endif	/* ROBOTS_H */

