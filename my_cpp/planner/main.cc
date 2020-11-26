#include <iostream>
#include "cyber/cyber.h"
#include "my_cpp/planner/lib/my_planner.h"

using apollo::my_planner::MyPlanner;

int main(int argc, char *argv[]) 
{
  apollo::cyber::Init(argv[0]);

  MyPlanner planner;
  planner.Init();

  apollo::cyber::WaitForShutdown();

  return 0;
}

