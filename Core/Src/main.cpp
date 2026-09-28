#include "cpp_main.h"
#include "gse_controller.h"

void cpp_main(void)
{
    // Statically allocate the controller so it persists in memory permanently
    static GseController controller;
    
    controller.Init();
    controller.Run();
}