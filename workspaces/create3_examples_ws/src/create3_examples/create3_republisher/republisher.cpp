// Copyright 2024 iRobot Corporation. All Rights Reserved.

#include <cassert>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "irobot_create_msgs/srv/e_stop.hpp"
#include "irobot_create_msgs/srv/reset_pose.hpp"
#include "irobot_create_msgs/srv/robot_power.hpp"

#include "irobot_create_msgs/action/audio_note_sequence.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/drive_arc.hpp"
#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/led_animation.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/wall_follow.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"

// Macros to reduce boilerplate
#define TRY_SETUP_SERVICE(type_arg) setup_service_if_matches<type_arg>(#type_arg, client_topic, server_topic, type)
#define TRY_SETUP_ACTION(type_arg) setup_action_if_matches<type_arg>(#type_arg, client_topic, server_topic, type)

class RepublisherNode : public rclcpp::Node
{
public:
    RepublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("create3_repub", options)
    {
        auto verbose_log_period_sec =
            this->declare_parameter("verbose_log_period_sec", rclcpp::ParameterValue(-1)).get<int>();
        m_services_timeout_sec =
            this->declare_parameter("services_timeout_sec", rclcpp::ParameterValue(60)).get<int>();
        m_actions_timeout_sec =
            this->declare_parameter("actions_timeout_sec", rclcpp::ParameterValue(600)).get<int>();
        m_actions_period_ms =
            this->declare_parameter("actions_period_ms", rclcpp::ParameterValue(50)).get<int>();

        if (verbose_log_period_sec > 0) {
            m_log_timer = this->create_wall_timer(std::chrono::seconds(verbose_log_period_sec), [this]() { this->debug_log(); });
        }

        const auto robot_namespace = this->get_robot_namespace();
        RCLCPP_INFO(
            this->get_logger(),
            "Creating republisher node with namespace '%s' to interact with robot '%s'",
            this->get_namespace(),
            robot_namespace.c_str());

        bool success = setup_republishers(robot_namespace);
        assert(success);

        RCLCPP_INFO(this->get_logger(), "Ready to go!");
    }

private:
    struct remapped_names_t
    {
        std::string robot_entity {};
        std::string this_entity {};
    };

    remapped_names_t get_remapped_names(std::string robot_namespace, const std::string & relative_name)
    {
        if (relative_name.empty() || relative_name[0] == '/') {
            throw std::runtime_error("Invalid relative_name name: " + relative_name);
        }

        if (robot_namespace.back() != '/') {
            robot_namespace += '/';
        }
        std::string this_namespace = this->get_namespace();
        if (this_namespace.back() != '/') {
            this_namespace += '/';
        }

        remapped_names_t remapped_names;
        remapped_names.robot_entity = robot_namespace + relative_name;
        remapped_names.this_entity = this_namespace + relative_name;
        return remapped_names;
    }

    bool setup_republishers(const std::string & robot_namespace)
    {
        auto robot_publications = get_entity_pairs("robot_publishers");
        for (const auto & [topic, type] : robot_publications) {
            auto names = get_remapped_names(robot_namespace, topic);
            setup_topic_republisher(names.robot_entity, names.this_entity, type);
        }
        auto robot_subscriptions = get_entity_pairs("robot_subscriptions");
        for (const auto & [topic, type] : robot_subscriptions) {
            auto names = get_remapped_names(robot_namespace, topic);
            setup_topic_republisher(names.this_entity, names.robot_entity, type);
        }
        auto robot_services = get_entity_pairs("robot_services");
        for (const auto & [service, type] : robot_services) {
            auto names = get_remapped_names(robot_namespace, service);
            setup_service_republisher(names.robot_entity, names.this_entity, type);
        }
        auto robot_actions = get_entity_pairs("robot_actions");
        for (const auto & [action, type] : robot_actions) {
            auto names = get_remapped_names(robot_namespace, action);
            setup_action_republisher(names.robot_entity, names.this_entity, type);
        }
        return true;
    }

    std::vector<std::pair<std::string, std::string>> get_entity_pairs(const std::string & param_name)
    {
        auto entity_list =
            this->declare_parameter(
                param_name,
                rclcpp::ParameterValue(std::vector<std::string>()))
                .get<std::vector<std::string>>();
        if (entity_list.size() % 2 != 0) {
            throw std::runtime_error(
                "Parameter must have an even number of elements: " + param_name + " found " + std::to_string(entity_list.size()));
        }
        std::vector<std::pair<std::string, std::string>> entity_pairs;
        // Use int iteration index to avoid wrap-around errors
        const int end_index = entity_list.size() - 1;
        for (int i = 0; i < end_index; i += 2) {
            entity_pairs.push_back(std::make_pair(entity_list[i], entity_list[i + 1]));
        }
        RCLCPP_INFO(this->get_logger(), "Found %ld entities for %s", entity_pairs.size(), param_name.c_str());
        return entity_pairs;
    }

    void setup_topic_republisher(
        const std::string & subscribed_topic,
        const std::string & published_topic,
        const std::string & topic_type)
    {
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s' and republishing it as '%s' with type '%s'",
            subscribed_topic.c_str(),
            published_topic.c_str(),
            topic_type.c_str());

        auto publisher = this->create_generic_publisher(
            published_topic,
            topic_type,
            rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
        
        auto subscriber = this->create_generic_subscription(
            subscribed_topic,
            topic_type,
            rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::BestEffort),
            [this, publisher=publisher](std::shared_ptr<rclcpp::SerializedMessage> message) {
                m_msg_counter++;
                ++m_topics_map[publisher->get_topic_name()];
                publisher->publish(*message);
            }
        );

        m_publishers.push_back(publisher);
        m_subscriptions.push_back(subscriber);
    }

    std::string cpp_type_to_name(const std::string & cpp_type)
    {
        // Converts "irobot_create_msgs::action::AudioNoteSequence" into "irobot_create_msgs/action/AudioNoteSequence"
        std::string type_name = cpp_type;
        const std::string to_replace = "::";
        const std::string replacement = "/";
        size_t pos = 0;
        while ((pos = type_name.find(to_replace, pos)) != std::string::npos) {
            type_name.replace(pos, to_replace.length(), replacement);
            pos += replacement.length();
        }
        return type_name;
    }

    template<typename ServiceT>
    bool setup_service_if_matches(const std::string & expected_type, const std::string & client_topic, const std::string & server_topic, const std::string & type)
    {
        if (cpp_type_to_name(expected_type) == type) {
            make_service_pair<ServiceT>(client_topic, server_topic);
            return true;
        }
        return false;
    }

    void setup_service_republisher(
        const std::string & client_topic,
        const std::string & server_topic,
        const std::string & type)
    {
        RCLCPP_INFO(this->get_logger(), "Remapping robot service '%s' as '%s' with type '%s'",
            client_topic.c_str(),
            server_topic.c_str(),
            type.c_str());

        bool setup_success =
            TRY_SETUP_SERVICE(irobot_create_msgs::srv::EStop) ||
            TRY_SETUP_SERVICE(irobot_create_msgs::srv::ResetPose) ||
            TRY_SETUP_SERVICE(irobot_create_msgs::srv::RobotPower);

        if (!setup_success) {
            throw std::runtime_error("Unrecognized service client " + client_topic);
        }
    }

    template<typename ActionT>
    bool setup_action_if_matches(const std::string & expected_type, const std::string & client_topic, const std::string & server_topic, const std::string & type)
    {
        if (cpp_type_to_name(expected_type) == type) {
            make_action_pair<ActionT>(client_topic, server_topic);
            return true;
        }
        return false;
    }

    void setup_action_republisher(
        const std::string & client_topic,
        const std::string & server_topic,
        const std::string & type)
    {
        RCLCPP_INFO(this->get_logger(), "Remapping robot action '%s' as '%s' with type '%s'",
            client_topic.c_str(),
            server_topic.c_str(),
            type.c_str());

        bool setup_success =
            TRY_SETUP_ACTION(irobot_create_msgs::action::AudioNoteSequence) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::Dock) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::DriveArc) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::DriveDistance) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::LedAnimation) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::NavigateToPosition) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::RotateAngle) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::Undock) ||
            TRY_SETUP_ACTION(irobot_create_msgs::action::WallFollow);

        if (!setup_success) {
            throw std::runtime_error("Unrecognized action client " + client_topic);
        }
    }

    template <typename ServiceT>
    void make_service_pair(const std::string & client_name, const std::string & server_name)
    {
        auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_callback_group(cb_group, this->get_node_base_interface());

        // We should do some thread management here but it's not a big deal.
        // The app will not shutdown nicely.
        auto executor_thread = std::thread([executor=executor](){
            executor->spin();
        });
        executor_thread.detach();

        // IMPORTANT: the client stays in the default callback group; the new one is only for the server
        auto client = this->create_client<ServiceT>(client_name, rmw_qos_profile_services_default, nullptr);
        auto server = this->create_service<ServiceT>(
            server_name,
            [this, client=client](typename ServiceT::Request::SharedPtr req, typename ServiceT::Response::SharedPtr res)
            {
                // Forward the request through our client and block for a response
                RCLCPP_INFO(this->get_logger(), "Forwarding service request to %s", client->get_service_name());
                auto future = client->async_send_request(req);
                if (future.wait_for(std::chrono::seconds(m_services_timeout_sec)) != std::future_status::ready) {
                    RCLCPP_ERROR(this->get_logger(), "Timed out service %s", client->get_service_name());
                    return;
                }
                res = future.get();
                RCLCPP_INFO(this->get_logger(), "Forwarding service response from %s", client->get_service_name());
            },
            rmw_qos_profile_services_default,
            cb_group);

        m_callback_groups.push_back(cb_group);
        m_clients.push_back(client);
        m_services.push_back(server);
    }

    template <typename ActionT>
    void make_action_pair(const std::string & client_name, const std::string & server_name)
    {
        auto action_timeout = std::chrono::seconds(m_actions_timeout_sec);
        auto action_period = std::chrono::milliseconds(m_actions_period_ms);
        auto client = rclcpp_action::create_client<ActionT>(this, client_name, nullptr);
        auto server = rclcpp_action::create_server<ActionT>(
            this, server_name,
            [client_name](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
            {
                std::cerr<<"Received action request for " << client_name << std::endl;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [client_name](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
            {
                std::cerr<<"Received action cancel request for " << client_name << std::endl;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [action_timeout, action_period, client=client, client_name](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
            {
                std::cerr<<"Forwarding action request to " << client_name << std::endl;
                auto action_thread = std::thread([action_timeout, action_period, client=client, user_goal_handle=handle, client_name]()
                {
                    const auto user_goal = user_goal_handle->get_goal();
                    auto start_time = std::chrono::high_resolution_clock::now();
                    auto robot_goal_handle_fut = client->async_send_goal(*user_goal);
                    while(true) {
                        if (std::chrono::high_resolution_clock::now() - start_time >= action_timeout) {
                            std::cerr << "WARNING: ROS 2 action " << client_name << " timed-out while it's still waiting for a goal handle from the robot" << std::endl;
                            user_goal_handle->abort(std::make_shared<typename ActionT::Result>());
                            return;
                        }
                        if (user_goal_handle->is_canceling()) {
                            std::cerr << "ERROR: Cancelling ROS 2 action " << client_name << " while it's still waiting for a goal handle from the robot" << std::endl;
                            user_goal_handle->canceled(std::make_shared<typename ActionT::Result>());
                            return;
                        }
                        if (robot_goal_handle_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                            break;
                        }
                        std::this_thread::sleep_for(action_period);
                    }

                    auto robot_goal_handle = robot_goal_handle_fut.get();
                    std::cerr << "Action request " << client_name << " received goal handle from the robot" << std::endl;
                    auto result_fut = client->async_get_result(robot_goal_handle);
                    while(true) {
                        if (std::chrono::high_resolution_clock::now() - start_time >= action_timeout) {
                            std::cerr << "WARNING: ROS 2 action " << client_name << " timed-out while running" << std::endl;
                            client->async_cancel_goal(robot_goal_handle);
                            user_goal_handle->abort(std::make_shared<typename ActionT::Result>());
                            return;
                        }
                        if (user_goal_handle->is_canceling()) {
                           std::cerr << "Cancelling ROS 2 action " << client_name << std::endl;
                            client->async_cancel_goal(robot_goal_handle);
                            user_goal_handle->canceled(std::make_shared<typename ActionT::Result>());
                            return;
                        }
                        if (result_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                            break;
                        }
                        std::this_thread::sleep_for(action_period);
                    }
                    auto wrapped_result = result_fut.get();
                    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        std::cerr << "Action request " << client_name << " SUCCESS received from the robot" << std::endl;
                        user_goal_handle->succeed(wrapped_result.result);
                    } else {
                        std::cerr << "Action request " << client_name << " ERROR " << static_cast<int>(wrapped_result.code) << " received from the robot" << std::endl;
                        user_goal_handle->abort(wrapped_result.result);
                    }
                });
                action_thread.detach();
            }
        );
        
        m_action_clients.push_back(client);
        m_action_servers.push_back(server);
    }

    std::string get_robot_namespace()
    {
        auto robot_namespace =
            this->declare_parameter("robot_namespace", rclcpp::ParameterValue("/")).get<std::string>();
        if (robot_namespace.empty()) {
            throw std::runtime_error("The 'robot_namespace' parameter can't be an empty string");
        }
        if (robot_namespace == this->get_namespace()) {
            throw std::runtime_error("The republisher node must have a different namespace from the robot!");
        }

        return robot_namespace;
    }

    void debug_log()
    {
        RCLCPP_INFO(this->get_logger(), "Total republished messages: %zu", m_msg_counter.load());
        for (const auto & [key, value] : m_topics_map) {
            RCLCPP_INFO(this->get_logger(), " - Topic '%s' count: %zu", key.c_str(), value);
        }
    }

    std::vector<std::shared_ptr<rclcpp::CallbackGroup>> m_callback_groups;
    std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> m_subscriptions;
    std::vector<std::shared_ptr<rclcpp::PublisherBase>> m_publishers;
    std::vector<std::shared_ptr<rclcpp::ClientBase>> m_clients;
    std::vector<std::shared_ptr<rclcpp::ServiceBase>> m_services;
    std::vector<std::shared_ptr<rclcpp_action::ClientBase>> m_action_clients;
    std::vector<std::shared_ptr<rclcpp_action::ServerBase>> m_action_servers;

    int m_services_timeout_sec {60};
    int m_actions_timeout_sec {600};
    int m_actions_period_ms {50};
    std::atomic<size_t> m_msg_counter {0};
    std::map<std::string, size_t> m_topics_map;

    rclcpp::TimerBase::SharedPtr m_log_timer;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<RepublisherNode>(options);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();

    rclcpp::shutdown();

    return 0;
}
