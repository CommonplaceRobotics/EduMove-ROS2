#pragma once
#include <thread>
#include <shared_mutex>
#include <condition_variable>

#include <stdio.h>
#include <sys/socket.h>

#include <vector>
#include <array>
#include <list>
#include <mutex>
#include <string>
#include <utility>

namespace irc_hardware
{   
    namespace PosInterfaceKeywords
    {
        const std::string START = "MSGSTART";
        const std::string END = "MSGEND";   
        const std::string CURRENTPOSITION = "Pos";
    }


    class FastPositionInterfaceSocket
    {
        private:
            std::string m_adress = "192.168.3.11"; 

            int sock; 

            int m_port = -1;

            int MessageBufferMaxLenght = 512;

            std::thread readThread;
            std::thread writeThread;

            std::mutex connectionLock;
            std::mutex messageLock;

            std::mutex m_ReadMutex;
            std::condition_variable_any m_PositionsInUpdated;
            
            std::mutex m_JointStateMutex;
            std::condition_variable_any m_JointStateUpdated;


            std::string m_messageBuffer;

            int m_interval;

            //Target Positions to publish 
            std::array<double, 9> joint_setPos_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            std::array<double, 6> cart_setPos_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

            //Current Positions received from iRC
            std::array<double, 6> currentintJoints = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            std::array<double, 3> currentextJoints = {0.0f, 0.0f, 0.0f};
            std::array<double, 6> currentCart = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

            int m_messageNumber;

            bool m_bStopRunning = true;
            bool m_bConnected = false;
            bool m_bListen; 
            bool m_bPublish = true;

            /**
            * @brief Attempts to connect to the PositionInterface at the given Address & port#
            * @param std::string ip  
            * @param int port
            */
            void Connect(const std::string & ip, int port);
            
            /**
            * @brief Adds a potentially partial message to the buffer and parses it
            * @param const char * buffer partial message
            *
            */
            void Parse(const char * buffer);

            /**
            * @Updates the 
            */
            void ProcessMsg(std::string & msg);
        public:

            bool m_bSend = false;


            FastPositionInterfaceSocket(const int interval);
            FastPositionInterfaceSocket();
            ~FastPositionInterfaceSocket();

            /**
            * @brief Thread that reads new incoming messages and updates the current Joint positions
            */
            void ReadThreadFunction();

            /**
            * @brief Thread that publishes the current target positions
            */
            void WriteThreadFunction();

            /**
            * @brief Starts the Interface by connecting and Starting threads
            * @param address 
            * @param interval - interval at which target positions should be published
            * @param port 
            */
            void Start(const std::string & address, int interval,  int port);

            /**
            * @brief Stops the Read & Write Threads
            */
            void Stop();
            
            /**
            * @brief Writes the current Joint positions and Cartesian positions into the given arrays
            * @param joints array containing positions of internal joints
            * @param externalJoints array containing position of additional external joints
            * @param cartPos array containing the current Cartesian pose
            */
            void GetLatestPositions(std::array<double, 6> & joints, std::array<double, 3> & externalJoints, std::array<double, 6> & cartPos);

            /**
            * @brief Updates the target positions of the joints
            * @param joints_ array containing joint positions (internal and external) 
            */
            void SetTargetJointPos(std::vector<double> & joints_, std::vector<double> & offset);

            /**
            * @brief Updates the target cartesian pose
            * @param cartPos_ array containg the target pose
            */
            void SetCartesianTargetPos(std::array<float, 6> & cartPos_);

            void Silence(bool silent);
    };
}