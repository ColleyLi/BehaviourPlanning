#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include "my_cpp/talker/proto/talker.pb.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::my_cpp::talker::proto::Message;

int main(int argc, char *argv[]) 
{
  apollo::cyber::Init(argv[0]);
  auto talker_node = apollo::cyber::CreateNode("talker");

  auto talker = talker_node->CreateWriter<Message>("channel/chatter");
  Rate rate(1.0);
  
  while (apollo::cyber::OK())
  {
    static uint64_t seq = 0;
    auto msg = std::make_shared<Message>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq++);
    msg->set_content("Hello, apollo!");
    
    talker->Write(msg);
    AINFO << "talker sent a message!";
    rate.Sleep();
  }
  return 0;
}
