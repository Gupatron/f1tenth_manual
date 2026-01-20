// ZenohCommunicator.hpp
#ifndef ZENOH_COMMUNICATOR_HPP
#define ZENOH_COMMUNICATOR_HPP

#include "zenoh.hxx"
#include <iostream>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include "Message.hpp"

/**
 * Handles Zenoh communication for rover control
 * Uses modern Zenoh 1.0+ C++ API
 */
class ZenohCommunicator {
private:
    zenoh::Session session_;
    zenoh::Publisher publisher_;
    std::unique_ptr<zenoh::Subscriber> subscriber_;
    
    std::string pub_key_;
    std::string sub_key_;
    
    // Callback for received messages
    std::function<void(const Message&)> message_callback_;
    
public:
    /**
     * Constructor
     * @param pub_key Key expression for publishing (e.g., "base/to/rover")
     * @param sub_key Key expression for subscribing (e.g., "rover/to/base")
     */
    ZenohCommunicator(const std::string& pub_key, const std::string& sub_key)
        : pub_key_(pub_key), sub_key_(sub_key) {
        
        try {
            // Open Zenoh session with default config
            auto config = zenoh::Config::create_default();
            session_ = zenoh::Session::open(std::move(config));
            
            // Declare publisher
            publisher_ = session_.declare_publisher(zenoh::KeyExpr(pub_key_));
            
            std::cout << "[Zenoh] Session opened successfully" << std::endl;
            std::cout << "[Zenoh] Publishing on: " << pub_key_ << std::endl;
            std::cout << "[Zenoh] Ready to subscribe to: " << sub_key_ << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "[Zenoh] Initialization error: " << e.what() << std::endl;
            throw;
        }
    }
    
    /**
     * Set callback for received messages and start subscriber
     */
    void setMessageCallback(std::function<void(const Message&)> callback) {
        message_callback_ = callback;
        
        try {
            // Declare subscriber with callback
            auto on_sample = [this](const zenoh::Sample& sample) {
                try {
                    // Get payload as bytes
                    auto payload = sample.get_payload();
                    auto bytes = payload.as_vector();
                    
                    // Deserialize message
                    if (bytes.size() >= 24) {
                        Message msg = Message::fromBinary(bytes.data(), bytes.size());
                        
                        // Call user callback
                        if (message_callback_) {
                            message_callback_(msg);
                        }
                    } else {
                        std::cerr << "[Zenoh] Received payload too small: " 
                                  << bytes.size() << " bytes" << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[Zenoh] Error in subscriber callback: " 
                              << e.what() << std::endl;
                }
            };
            
            // Create subscriber
            subscriber_ = std::make_unique<zenoh::Subscriber>(
                session_.declare_subscriber(zenoh::KeyExpr(sub_key_), std::move(on_sample))
            );
            
            std::cout << "[Zenoh] Subscribed to: " << sub_key_ << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "[Zenoh] Subscriber setup error: " << e.what() << std::endl;
            throw;
        }
    }
    
    /**
     * Publish a message
     */
    void publish(const Message& msg) {
        try {
            auto binary = msg.toBinary();
            
            // Create Zenoh bytes from binary data
            zenoh::Bytes payload(binary.data(), binary.size());
            
            // Publish
            publisher_.put(std::move(payload));
            
        } catch (const std::exception& e) {
            std::cerr << "[Zenoh] Error publishing: " << e.what() << std::endl;
        }
    }
    
    /**
     * Check if communicator is ready
     */
    bool isReady() const {
        try {
            // Simple check - if we got here, initialization succeeded
            return true;
        } catch (...) {
            return false;
        }
    }
    
    ~ZenohCommunicator() {
        try {
            std::cout << "[Zenoh] Closing session" << std::endl;
        } catch (...) {
            // Suppress exceptions in destructor
        }
    }
};

#endif // ZENOH_COMMUNICATOR_HPP