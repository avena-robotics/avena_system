#include "bullet_server/bullet_server.hpp"

namespace bullet_server
{

    BulletServer::BulletServer(const rclcpp::NodeOptions &options)
        : Node("bullet_server", options)
    {
// Right now I do not know why Bullet needs this but for now keep it here
#ifndef _WIN32
        struct sigaction action;
        memset(&action, 0x0, sizeof(action));
        action.sa_handler = cleanup;
        static const int signos[] = {SIGHUP, SIGINT, SIGQUIT, SIGABRT, SIGSEGV, SIGPIPE, SIGTERM};
        for (size_t ii(0); ii < sizeof(signos) / sizeof(*signos); ++ii)
        {
            if (0 != sigaction(signos[ii], &action, NULL))
            {
                err(EXIT_FAILURE, "signal %d", signos[ii]);
            }
        }
#endif

        _server_thread = std::thread([]()
                                     {
                                         DummyGUIHelper noGfx;

                                         CommonExampleOptions commons_example_options(&noGfx);

                                         // args.GetCmdLineArgument("shared_memory_key", gSharedMemoryKey);
                                         // args.GetCmdLineArgument("sharedMemoryKey", gSharedMemoryKey);

                                         // commons_example_options.m_option |= PHYSICS_SERVER_ENABLE_COMMAND_LOGGING;
                                         // commons_example_options.m_option |= PHYSICS_SERVER_REPLAY_FROM_COMMAND_LOG;

                                         example = (SharedMemoryCommon *)PhysicsServerCreateFuncBullet2(commons_example_options);

                                         example->initPhysics();

                                         while (example->isConnected() && !(example->wantsTermination() || interrupted))
                                         {
                                             example->stepSimulation(1.f / 60.f);
                                         }

                                         example->exitPhysics();

                                         delete example;
                                     });
    }

    BulletServer::~BulletServer()
    {
        if (_server_thread.joinable())
            _server_thread.join();
    }

} // namespace bullet_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bullet_server::BulletServer)