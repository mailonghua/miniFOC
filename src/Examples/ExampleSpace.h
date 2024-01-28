#ifndef __EXAMPLE_SPACE_H__
#define __EXAMPLE_SPACE_H__
#include <SimpleFOC.h>
#include "driver/i2c.h"

namespace EX{
    /*I2c*/
    void i2c_setup();
    void i2c_loop();
    /*SimpleFOC*/
    void FocOpenLoopSetup();
    void FocOpenLoopLoop();
    void FocCurrentLoopSetup();
    void FocCurrentLoopLoop(uint32_t sleepms);
    //
}
#endif