#include "irc_ros_hardware/FastPositionInterface/position_interface_socket.hpp"
#include "irc_ros_hardware/CRI/cri_keywords.hpp"

#include <arpa/inet.h>
#include <unistd.h>
#include <shared_mutex>

#include <math.h>
#include "rclcpp/rclcpp.hpp"


namespace irc_hardware
{
    
    FastPositionInterfaceSocket::FastPositionInterfaceSocket(){};

    FastPositionInterfaceSocket::FastPositionInterfaceSocket(const int interval)
    {
        m_interval = interval;
    }

    FastPositionInterfaceSocket::~FastPositionInterfaceSocket()
    {
        Stop();
    }

    /**
    * @brief Attempts to connect to the PositionInterface at the given Address & port#
    * @param std::string ip  
    * @param int port
    */
    void FastPositionInterfaceSocket::Connect(const std::string & ip, int port)
    {
        // Make sure that we do not try to establish the same connection multiple times
        // at the same time.
        std::unique_lock<std::mutex> lock(connectionLock);

        while (!m_bConnected) 
        {
            sock = 0;
            struct sockaddr_in serv_addr;

            if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CRI"), "Socket creation error.");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(port);

            // Convert IPv4 and IPv6 addresses from text to binary form
            if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
                RCLCPP_ERROR(
                rclcpp::get_logger("iRC_ROS::CRI"), "Invalid robot IP address / Address not supported.");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CRI"), "Connection Failed.");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            m_bConnected = true;
            RCLCPP_INFO(
                rclcpp::get_logger("iRC_ROS::CRI"), "Connected to FastPositionInterface at %s:%d", ip.c_str(), port);
        }
    
    }
    


    void FastPositionInterfaceSocket::ProcessMsg(std::string & msg)
    {      
        std::string::size_type posIdx = msg.find("Pos");
        if (posIdx == 0)
        {
            std::string::size_type iJointsStart = msg.find("J");
            std::string::size_type eJointsStart = msg.find("E");
            std::string::size_type cartesianStart = msg.find("C");

            if(!((iJointsStart < eJointsStart) && (eJointsStart < cartesianStart)))
            {   
                return;
            }
            // std::string iJointsString = Substring(temp, iJointsStart, eJointsStart, 1);
            if(iJointsStart != std::string::npos)
            {
                std::array<double, 6> tempJoints = {0};
                std::stringstream ss(msg.substr(iJointsStart + 2));
                for(size_t i = 0; i < tempJoints.size(); i++)
                {
                    double value = 0.0;
                    ss >> value;
                    if (!ss.fail())
                    {
                        tempJoints[i] = value;
                    }                        
                }
                {                            
                    std::unique_lock<std::mutex> lock(m_ReadMutex);
                    for(int i = 0; i < currentintJoints.size(); i++)
                    {
                        currentintJoints[i] = tempJoints[i];
                    }
                    // RCLCPP_INFO(rclcpp::get_logger("FastPosParse"), "%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", currentintJoints[0], currentintJoints[1], currentintJoints[2], currentintJoints[3], currentintJoints[4], currentintJoints[5]);
                    m_PositionsInUpdated.notify_one();
                }                            
            }           
        }     
    }

    /**
    * @brief Adds a potentially partial message to the buffer and parses it
    * @param const char * buffer partial message
    */
    void FastPositionInterfaceSocket::Parse(const char * buffer)
    {
        if (m_messageBuffer.empty())
            m_messageBuffer = buffer;
        else
            m_messageBuffer += buffer;

        // shorten the buffer if it is too long
        if (m_messageBuffer.size() > MessageBufferMaxLenght)
        {
            m_messageBuffer = m_messageBuffer.substr(m_messageBuffer.size() - MessageBufferMaxLenght, MessageBufferMaxLenght);
        }    
        
        while (!m_messageBuffer.empty())
        {   
            size_t startIdx = m_messageBuffer.find("MSGSTART");

            if (startIdx != std::string::npos)
            {
                size_t endIdx = m_messageBuffer.find("MSGEND");
                if(endIdx != std::string::npos)
                {   
                    
                    std::string temp = m_messageBuffer.substr(startIdx + 9, endIdx - startIdx -10);

                    ProcessMsg(temp);  
                    //remove processed msg from buffer
                    m_messageBuffer = m_messageBuffer.substr(endIdx + 6);                 
                }
                else
                {
                    break;
                }            
            }
            else
            {
                break;
            }
        }
    }

       
    /**
    * @brief Thread that reads new incoming messages and updates the current Joint positions
    */
    void FastPositionInterfaceSocket::ReadThreadFunction()
    {
        char buffer[MessageBufferMaxLenght] = {0};

        int debugCnt = 0;
        while(m_bConnected && !m_bStopRunning)
        {
            std::fill_n(buffer, MessageBufferMaxLenght, '\0');
            
            
            int valread = read(sock, buffer, MessageBufferMaxLenght);

            if (valread == 0)
            {
                //ERROR / KEINE NACHRICHT 
            }
            else
            {   
                Parse(buffer);
            }
            debugCnt++;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    /**
    * @brief Thread that publishes the current target positions
    */
    void FastPositionInterfaceSocket::WriteThreadFunction()
    {   
        int debugCnt = 0;
        while(m_bConnected && !m_bStopRunning)
        {        

            if (m_bPublish)
            {
                std::stringstream posMsgOut;
                posMsgOut.precision(10);

                posMsgOut << "MSGSTART "
                          << "Pos "
                          << "J ";
                {
                    std::unique_lock<std::mutex> lock(m_JointStateMutex);
                    m_JointStateUpdated.wait_for(lock, std::chrono::milliseconds(200));
                    for (int i=0; i<6; i++)
                    {   
                        posMsgOut << joint_setPos_[i] 
                                << " ";
                    }
                }            
                posMsgOut << "MSGEND";
                       
                std::string posMsgString = posMsgOut.str();
                int sent = send(sock, posMsgString.c_str(), posMsgString.length(), 0);

                if (sent < 0)
                {
                    m_bConnected = false;
                }
            
                if (debugCnt % 100 == 0) RCLCPP_INFO(rclcpp::get_logger("iRC_ROS::FastPosInterface"), "%s", posMsgString.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(m_interval));
                debugCnt++;            
            }
            
        }
    }

    /**
    * @brief Starts the Interface by connecting and Starting threads
    * @param address 
    * @param interval - interval at which target positions should be published
    * @param port 
    */
    void FastPositionInterfaceSocket::Start(const std::string & address, int interval,  int port)
    {        
        Connect(address, port);

        m_bStopRunning = false;

        m_interval = interval;
        
        readThread = std::thread(&FastPositionInterfaceSocket::ReadThreadFunction, this);
        writeThread = std::thread(&FastPositionInterfaceSocket::WriteThreadFunction, this);
    }


    /**
    * @brief Stops the Read & Write Threads
    */
    void FastPositionInterfaceSocket::Stop()
    {
        m_bConnected = false;
        m_bStopRunning = true;

        if (readThread.joinable()) 
        {
            readThread.join();
        }
        if(writeThread.joinable())
        {
            writeThread.join();
        }        
    }
    
    /**
    * @brief Writes the current Joint positions and Cartesian positions into the given arrays
    * @param joints array containing positions of internal joints
    * @param externalJoints array containing position of additional external joints
    * @param cartPos array containing the current Cartesian pose
    */
    void FastPositionInterfaceSocket::GetLatestPositions(std::array<double, 6> & joints, std::array<double, 3> & externalJoints, std::array<double, 6> & cartPos)
    {
        std::unique_lock<std::mutex> lock(m_ReadMutex);
        m_PositionsInUpdated.wait_for(lock, std::chrono::milliseconds(200));

        for(size_t i=0; i<currentintJoints.size(); i++)
        {
            joints[i] = currentintJoints[i];
        }

        for(size_t i=0; i<currentCart.size(); i++)
        {
            cartPos[i] = currentCart[i];
        }

        for(size_t i=0; i<currentextJoints.size(); i++)
        {
            externalJoints[i] = currentextJoints[i];
        }
    }

    /**
    * @brief Updates the target positions of the joints
    * @param joints_ array containing joint positions (internal and external) 
    */
    void FastPositionInterfaceSocket::SetTargetJointPos(std::vector<double> & joints_, std::vector<double> & offset)
    {
        std::unique_lock<std::mutex> lock(m_JointStateMutex);
               
        for (size_t i=0; i<joints_.size(); i++)
        {   
            if(i <= offset.size())
            {
                joint_setPos_[i] = ( joints_[i]  * 180 /M_PI) - offset[i];
            }
            else
            {
                joint_setPos_[i] = joints_[i];
            }
            if (i >= 9) break;
        }

        // RCLCPP_INFO(rclcpp::get_logger("PosInterface"), " \n A1:%0.2f, A2:%0.2f, A3:%0.2f, A4:%0.2f, A5:%0.2f, A6:%0.2f \n A1:%0.2f, A2:%0.2f, A3:%0.2f, A4:%0.2f, A5:%0.2f, A6:%0.2f"
        //                                                 , joint_setPos_[0], joint_setPos_[1], joint_setPos_[2], joint_setPos_[3], joint_setPos_[4], joint_setPos_[5]
        //                                                 , currentintJoints[0], currentintJoints[1], currentintJoints[2], currentintJoints[3], currentintJoints[4], currentintJoints[5]);
        m_JointStateUpdated.notify_one();
    }

    /**
    * @brief Updates the target cartesian pose
    * @param cartPos_ array containg the target pose
    */
    void FastPositionInterfaceSocket::SetCartesianTargetPos(std::array<float, 6> & cartPos_)
    {
        std::unique_lock<std::mutex> lock(m_JointStateMutex);

        for (size_t i=0; i<cartPos_.size(); i++)
        {
            cart_setPos_[i] = cartPos_[i];
        }
    }

    /**
    * @brief Activates / Deactivates publishing of target positions
    * @param silent true publisher deactivated 
    *               false publisher activated
    */
    void FastPositionInterfaceSocket::Silence(bool silent)
    {
        m_bPublish = !silent;
    }
    
    
}