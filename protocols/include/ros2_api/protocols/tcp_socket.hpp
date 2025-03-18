#include <ros2_api/protocols/zeromq.hpp>
#include <yaml-cpp/yaml.h>

namespace protocols {

    class TCPSocket : public ZeroMQ
    {
    public:
      void initialize(const YAML::Node &config) override;
    };
} // namespace protocols