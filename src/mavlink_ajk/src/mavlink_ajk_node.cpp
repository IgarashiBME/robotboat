/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C++11
 * Robot Operating System Kinetic
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
**/

/* C++ standard library */
#include "stdio.h"
#include "errno.h"
#include "string.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "unistd.h"
#include "stdlib.h"
#include "fcntl.h"
#include "time.h"
#include "sys/time.h"
#include "arpa/inet.h"
#include "iostream"
#include "fstream"

/* ROS library */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt16.h"

/* custom ROS messages */
#include "mavlink_ajk/MAV_Mission.h"
#include "mavlink_ajk/MAV_Modes.h"
#include "mavlink_ajk/MAV_Joystick.h"
#include "look_ahead/Auto_Log.h"
#include "ubx_analyzer/RELPOSNED.h"
#include "ubx_analyzer/UTMHP.h"
#include "mavlink_ajk/NavPVT.h"

/* mavlink library */
#include "mavlink.h"

/* buffer size */
#define BUFFER_LENGTH 300

/* mavlink ARDUPILOTMEGA's custom number for BASE_MODE */
#define ARDUPILOT_GUIDED_ARMED 217
#define ARDUPILOT_GUIDED_DISARMED 89

/* ESTIMATOR_STATUS_FLAG */
#define GOOD_ESTIMATOR_ATTITUDE 1

/* Interval */
#define COMMON_INTERVAL 1000000
#define LIGHT_INTERVAL 100000

/* Joystick */
#define JOY_YAW_NEUTRAL 0
#define JOY_TH_NEUTRAL 500
#define JOY_THRESHOLD 10
#define NOT_SAVED_PARAM "shutdown"
using namespace std;

uint64_t microsSinceEpoch();
int sock;

class Listener{
public:
    void gnss_callback(const ubx_analyzer::UTMHP::ConstPtr& msg);
    /* sanyokiki lat 34.500682  lon 133.558131
       yayoi     lat 35.716761  lon 139.761254
       tanashi   lat 35.736805  lon 139.539676
       osakaike  lat 34.559582  lon 133.537000*/
    int lat = 35.736805 * 10000000;  // latitude
    int lon = 139.539676 * 10000000;  // longitude
    //int lat = 34.500682 * 10000000;
    //int lon = 133.558131 * 10000000;
    int alt = 10000;  // altitude above elliposid
    int fix_type = GPS_FIX_TYPE_RTK_FIXED;
    int satellites = 0; // number of satellites visible. If unknown, set to 255.

    void auto_log_callback(const look_ahead::Auto_Log::ConstPtr& msg);
    int current_seq = 0;
    float cross_track_error = 0;
    float angular_z = 0;

    void move_base_callback(const ubx_analyzer::RELPOSNED::ConstPtr& msg);
    double yaw;
    uint16_t move_base_fix_status = 0;
};

class QGC_parameter{
public:
    void parameter_getter();
    /* rosparameters */
    float Kp_value;
    float Ki_value;
    float Kd_value;
    float look_ahead_value;
    float i_control_dist_value;
    float i_limit_value;
    float linear_velocity_value;
};

void Listener::gnss_callback(const ubx_analyzer::UTMHP::ConstPtr& msg){
    lat = msg->latHp * 10000000;
    lon = msg->lonHp * 10000000;
    satellites = msg->numSV;

    switch(msg->fix_status){
        case 2:
            fix_type = GPS_FIX_TYPE_RTK_FIXED;
            break;
        case 1:
            fix_type = GPS_FIX_TYPE_RTK_FLOAT;
            break;
        case 0:
            fix_type = GPS_FIX_TYPE_DGPS;
            break;
    }
    //ROS_INFO("info [%f]", msg->lat);
}

void Listener::auto_log_callback(const look_ahead::Auto_Log::ConstPtr& msg){
    current_seq = msg->waypoint_seq;
    cross_track_error = msg->cross_track_error;
    angular_z = msg->angular_z;
}

void Listener::move_base_callback(const ubx_analyzer::RELPOSNED::ConstPtr& msg){
    yaw = msg->QGC_heading;
    switch(msg->fix_status){
        case 2:
            move_base_fix_status = GOOD_ESTIMATOR_ATTITUDE;
            break;
        case 1:
            move_base_fix_status = 0;
            break;
        case 0:
            move_base_fix_status = 0;
            break;
    }
}

void QGC_parameter::parameter_getter(){
    ros::param::get("/mavlink_ajk/Kp", Kp_value);
    ros::param::get("/mavlink_ajk/Ki", Ki_value);
    ros::param::get("/mavlink_ajk/Kd", Kd_value);
    ros::param::get("/mavlink_ajk/look_ahead", look_ahead_value);
    ros::param::get("/mavlink_ajk/i_control_dist", i_control_dist_value);
    ros::param::get("/mavlink_ajk/i_limit",  i_limit_value);
    ros::param::get("/mavlink_ajk/linear_velocity",  linear_velocity_value);
}

int main(int argc, char **argv){
    char target_ip[100];
	
    float position[6] = {};
    struct sockaddr_in gcAddr; 
    struct sockaddr_in locAddr;
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;
    int bytes_sent;
    mavlink_message_t mavmsg;
    uint16_t len;
    int i = 0;
    unsigned int temp = 0;

    bool is_mission_req = false;
    int mission_total_seq = 0;
    unsigned int mission_seq = 0;
    int pre_mission_seq = -1;

    /* mavlink mode */
    uint64_t base_mode = 0; //MAV_MODE_GUIDED_DISARMED;
    uint64_t custom_mode = 0;
    bool mission_start = false;

    /* time interval */
    uint64_t pre_heartbeat_time;
    int heartbeat_interval = 900000; //0.9 second

    uint64_t pre_request_time;
    int request_interval = 30000; // 0.05 second

    uint64_t last_gcs_heartbeat_time;
    int gcs_heartbeat_interval = 5000000; // 5 second

    /* parameter protocol */
    bool parameter_set = false;
    char* parameter_id;
    float parameter_value;
    uint8_t parameter_type;
    uint16_t parameter_count;
    uint16_t parameter_index;

    /* manual_control */
    uint8_t manual_target;
    int16_t manual_x;
    int16_t manual_y;
    int16_t manual_z;
    int16_t manual_r;
    uint16_t manual_buttons;

    /* ros intializer */
    ros::init(argc, argv, "mavlink_node");
    ROS_INFO("fake fcu start");
    ros::NodeHandle n;

    Listener listener;
    QGC_parameter qgc_param;
    ros::Subscriber sub = n.subscribe("/utm_hp", 10, &Listener::gnss_callback, &listener);
    ros::Subscriber auto_log = n.subscribe("/auto_log", 1, &Listener::auto_log_callback, &listener);
    ros::Subscriber move_base = n.subscribe("/relposned", 1, &Listener::move_base_callback, &listener);

    ros::Publisher pub_mission = n.advertise<mavlink_ajk::MAV_Mission>("/mav/mission", 1000);
    mavlink_ajk::MAV_Mission mission_rosmsg;

    ros::Publisher pub_modes = n.advertise<mavlink_ajk::MAV_Modes>("/mav/modes", 1);
    mavlink_ajk::MAV_Modes modes_rosmsg;

    ros::Publisher pub_joystick = n.advertise<mavlink_ajk::MAV_Joystick>("/mav/joystick", 1);
    mavlink_ajk::MAV_Joystick joystick_rosmsg;

    ros::Publisher pub_mission_set_current = n.advertise<std_msgs::UInt16>("/mav/mission_set_current", 10);

    while (!ros::ok());

    /* ROS parameters */
    std::string param_path;
    ros::param::get("~param_path", param_path);
    ros::param::get("/mission_total_seq", mission_total_seq);
    qgc_param.parameter_getter();
    //printf("%i", mission_total_seq);
    char rosdump_cmd[100];
    sprintf(rosdump_cmd, "rosparam dump -v %s /mavlink_ajk", param_path.c_str());
    system(rosdump_cmd);

    if ((sock=socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
        perror("error");
        exit(EXIT_FAILURE);
    }

    // Change the target ip if parameter was given
    strcpy(target_ip, "127.0.0.1");
    if (argc == 2){
        strcpy(target_ip, argv[1]);
    }

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14551);
	
    /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
    if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr))){
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    } 
	
    /* Attempt to make it non blocking */
    if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0){
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }

    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr(target_ip);
    gcAddr.sin_port = htons(14550);

    /* time setting for interval */
    pre_heartbeat_time = microsSinceEpoch();
    pre_request_time = microsSinceEpoch();

    while (ros::ok()){
        /* time interval */
        if (microsSinceEpoch() - pre_heartbeat_time > heartbeat_interval){
            /* print */
            //std::cout << microsSinceEpoch() - pre_time << std::endl;
            //std::cout << listener.fix_type << std::endl; 

            /*Send Heartbeat */
            mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                       base_mode, custom_mode, MAV_STATE_ACTIVE);
            //mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_PX4, 
            //                           base_mode, custom_mode, MAV_STATE_ACTIVE);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            /* Send Status */
            mavlink_msg_sys_status_pack(1, 200, &mavmsg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));

            /* Send Local Position */
            mavlink_msg_local_position_ned_pack(1, 200, &mavmsg, microsSinceEpoch(), 
                                                position[0], position[1], position[2],
                                                position[3], position[4], position[5]);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
            /* Send attitude */ 
            mavlink_msg_attitude_pack(1, 200, &mavmsg, microsSinceEpoch(), 0, 0, listener.yaw, 0.01, 0.02, 0.03);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
            /* Send GPS */
            mavlink_msg_gps_raw_int_pack(1, 200, &mavmsg, 0, listener.fix_type, 
                                         listener.lat, listener.lon, listener.alt, 65535, 65535, 
                                         65535, 65535, listener.satellites);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            /* Send ESTIMATOR_STATUS */
            mavlink_msg_estimator_status_pack(1, 200, &mavmsg, microsSinceEpoch(), listener.move_base_fix_status,
                                              listener.angular_z, 0, 0, 0, 0, 0, listener.cross_track_error, 0);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            pre_heartbeat_time = microsSinceEpoch();

            /* publish ARM-DISARM ROS message */
            modes_rosmsg.base_mode = base_mode;
            modes_rosmsg.custom_mode = custom_mode;
            modes_rosmsg.mission_start = mission_start;
            pub_modes.publish(modes_rosmsg);

            /* send mission_current */
            std::cout << listener.current_seq << std::endl;
            std::cout << mission_total_seq << std::endl;
            if (base_mode == ARDUPILOT_GUIDED_ARMED){
                if (listener.current_seq > mission_total_seq){
                    base_mode = ARDUPILOT_GUIDED_DISARMED;
                    mission_start = false;
                }
                mavlink_msg_mission_current_pack(1, 200, &mavmsg, listener.current_seq);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
            }
        }

        /* when the gcs heartbeat was stopped, then DISARM and mission_start is False */
        //if (microsSinceEpoch() - last_gcs_heartbeat_time > gcs_heartbeat_interval){
        //    usleep(COMMON_INTERVAL);
        //    ROS_INFO("GCS signal was interrupted");
        //    //std::cout << "GCS signal was interrupted\n" << std::endl;
        //    base_mode = ARDUPILOT_GUIDED_DISARMED;
        //    mission_start = false;            
        //}

        /* Mission Request */
        if (is_mission_req == true && microsSinceEpoch() - pre_request_time > request_interval){
            //printf("request\n");
            //std::cout << microsSinceEpoch() - pre_request_time << std::endl;
            mavlink_msg_mission_request_int_pack(1, 200, &mavmsg, 0, 0, mission_seq);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            pre_request_time = microsSinceEpoch();
        }

        /* response for PARAM_SET  */
        if (parameter_set == true){
            usleep(COMMON_INTERVAL);
            ROS_INFO("send param_value");
            printf("%s, %f, %i\n", parameter_id, parameter_value, parameter_type);
            mavlink_msg_param_value_pack(1, 1, &mavmsg, parameter_id, parameter_value, parameter_type,
                                         1, 0);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
            parameter_set = false;

            char param_cmd[100];
            sprintf(param_cmd, "/mavlink_ajk/%s", parameter_id);
            ros::param::set(param_cmd, parameter_value);

            if (strcmp(parameter_id, NOT_SAVED_PARAM)){
                sprintf(rosdump_cmd, "rosparam dump -v %s /mavlink_ajk", param_path.c_str());
                system(rosdump_cmd);
            }
        }

        /* receiver section */
        memset(buf, 0, BUFFER_LENGTH);
        recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

        if (recsize > 0){
            // Something received - print out all bytes and parse packet
            mavlink_message_t mavmsg;
            mavlink_status_t status;

            //printf("Bytes Received: %d\nDatagram: ", (int)recsize);

            for (i = 0; i < recsize; ++i){
                temp = buf[i];
                /* Packet received */
                //printf("%02x ", (unsigned char)temp);
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mavmsg, &status)){
                    // Packet decode
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                           mavmsg.sysid, mavmsg.compid, mavmsg.len, mavmsg.msgid);
                }
            }
            switch(mavmsg.msgid){
            case MAVLINK_MSG_ID_HEARTBEAT:{ //MAV ID 0
                last_gcs_heartbeat_time = microsSinceEpoch();
                break;
            }

            /* SET_MODE decoder */
            case MAVLINK_MSG_ID_SET_MODE:{ // MAV ID 11
                mavlink_set_mode_t mavsm;

                // decode SET_MODE message
                mavlink_msg_set_mode_decode(&mavmsg, &mavsm);
                printf("%i, %i, %i", mavsm.custom_mode, mavsm.target_system, mavsm.base_mode);
                custom_mode = mavsm.custom_mode;
                base_mode = mavsm.base_mode;
                break;
            }

            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:{ // MAV ID 21
                ROS_INFO("param request received");
                qgc_param.parameter_getter();
                usleep(LIGHT_INTERVAL);

                mavlink_msg_param_value_pack(1, 1, &mavmsg, "Kp", qgc_param.Kp_value, MAVLINK_TYPE_FLOAT, 7, 0);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
                usleep(LIGHT_INTERVAL);
                mavlink_msg_param_value_pack(1, 1, &mavmsg, "Ki", qgc_param.Ki_value, MAVLINK_TYPE_FLOAT, 7, 1);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
                usleep(LIGHT_INTERVAL);
                mavlink_msg_param_value_pack(1, 1, &mavmsg, "Kd", qgc_param.Kd_value, MAVLINK_TYPE_FLOAT, 7, 2);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
                usleep(LIGHT_INTERVAL);
                mavlink_msg_param_value_pack(1, 1, &mavmsg, "look_ahead", qgc_param.look_ahead_value, MAVLINK_TYPE_FLOAT, 7, 3);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
                usleep(LIGHT_INTERVAL);
                mavlink_msg_param_value_pack(1, 1, &mavmsg, "i_control_dist", qgc_param.i_control_dist_value, MAVLINK_TYPE_FLOAT, 7, 4);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
                usleep(LIGHT_INTERVAL);
                mavlink_msg_param_value_pack(1, 1, &mavmsg, "i_limit", qgc_param.i_limit_value, MAVLINK_TYPE_FLOAT, 7, 5);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
                usleep(LIGHT_INTERVAL);
                mavlink_msg_param_value_pack(1, 1, &mavmsg, "linear_velocity", qgc_param.linear_velocity_value, MAVLINK_TYPE_FLOAT, 7, 6);
                len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                    sizeof(struct sockaddr_in));
                break;
            }

            case MAVLINK_MSG_ID_PARAM_SET:{ // MAV ID 23
                mavlink_param_set_t mavps;
                ROS_INFO("param_set received");                
                
                mavlink_msg_param_set_decode(&mavmsg, &mavps);
                parameter_set = true;
                parameter_id = mavps.param_id;
                parameter_value = mavps.param_value;
                parameter_type = mavps.param_type;
                break;
            }

            case MAVLINK_MSG_ID_MISSION_SET_CURRENT:{ // MAV ID 41
                mavlink_mission_set_current_t mavmsc;
                std_msgs::UInt16 mission_set_current;
                    
                mavlink_msg_mission_set_current_decode(&mavmsg, &mavmsc);
                mission_set_current.data = mavmsc.seq;

                pub_mission_set_current.publish(mission_set_current);
                printf("mission_set_current: %i\n", mavmsc.seq);
                break;
            }

            case MAVLINK_MSG_ID_MISSION_COUNT:{ // MAV ID 44
                mavlink_mission_count_t mavmc;
                printf("mission count was received\n");
                    
                mavlink_msg_mission_count_decode(&mavmsg, &mavmc);
                is_mission_req = true;
                mission_total_seq = mavmc.count;
                mission_seq = 0;
                printf("%i\n", mission_total_seq);
                break;
            }

            /* manual control (joystick) */
            case MAVLINK_MSG_ID_MANUAL_CONTROL:{ // MAV ID 69
                mavlink_manual_control_t mavmc;
                    
                mavlink_msg_manual_control_decode(&mavmsg, &mavmc);
                manual_target = mavmc.target;
                manual_x = mavmc.x;
                manual_y = mavmc.y;
                manual_z = mavmc.z;
                manual_r = mavmc.r;
                manual_buttons = mavmc.buttons;
                printf("target:%i, pitch:%i, roll:%i, Throttle:%i, Yaw:%i, buttons:%i\n", 
                       manual_target, manual_x, manual_y, manual_z, manual_r, manual_buttons);

                /* publish ARM-DISARM ROS message */
                //if (JOY_THRESHOLD < std::abs(manual_z) - JOY_TH_NEUTRAL || 
                //    JOY_THRESHOLD < std::abs(manual_r) - JOY_YAW_NEUTRAL){
                //    joystick_rosmsg.stamp = ros::Time::now();
                //    joystick_rosmsg.roll = manual_y;
                //    joystick_rosmsg.pitch = manual_x;
                //    joystick_rosmsg.yaw = manual_r;
                //    joystick_rosmsg.throttle = manual_z;
                //    pub_joystick.publish(joystick_rosmsg);
                //}
                joystick_rosmsg.stamp = ros::Time::now();
                joystick_rosmsg.roll = manual_y;
                joystick_rosmsg.pitch = manual_x;
                joystick_rosmsg.yaw = manual_r;
                joystick_rosmsg.throttle = manual_z;
                joystick_rosmsg.buttons = mavmc.buttons;
                pub_joystick.publish(joystick_rosmsg);
                break;
            }

            /* mission receiver */
            case MAVLINK_MSG_ID_MISSION_ITEM_INT:{ // MAV ID 73
                mavlink_mission_item_int_t mavmii;

                //printf("mission item was received\n");

                // decode MISSION_ITEM_INT message
                mavlink_msg_mission_item_int_decode(&mavmsg, &mavmii);
                double waypoint_x = mavmii.x/10000000.0;
                double waypoint_y = mavmii.y/10000000.0;
                    
                // print and output waypoint
                if (pre_mission_seq != mavmii.seq && mission_seq == mavmii.seq){
                    printf("%i, %i, %i, %.09f, %.09f\n", mavmii.seq, mission_total_seq, mavmii.command,
                           waypoint_x, waypoint_y);
                    //fstream fs;
                    //fs.open("/home/nouki/waypoint.csv", ios::out | ios::app);
                    //fs << mission_seq << ",";
                    //fs << fixed << setprecision(8) << waypoint_x << "," << waypoint_y << endl; 
                    //fs.close();

                    // publish ROS message
                    mission_rosmsg.seq = mavmii.seq;
                    mission_rosmsg.total_seq = mission_total_seq;
                    mission_rosmsg.command = mavmii.command;
                    mission_rosmsg.latitude = waypoint_x;
                    mission_rosmsg.longitude = waypoint_y;
                    pub_mission.publish(mission_rosmsg);

                    pre_mission_seq = mavmii.seq;
                    mission_seq = mavmii.seq+1;
                }

                // if mission sequence is end, send mission ack
                if (is_mission_req == true && mission_seq == mission_total_seq){
                    mavlink_msg_mission_ack_pack(1, 200, &mavmsg, 0, 0, MAV_MISSION_TYPE_MISSION);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, 
                                        sizeof(struct sockaddr_in));

                    ros::param::set("/mission_total_seq", mission_total_seq);                    

                    // mission request is end
                    is_mission_req = false;
                }
                break;
            }

            /* COMMAND_LONG decoder */
            case MAVLINK_MSG_ID_COMMAND_LONG:{ //MAV ID 76
                mavlink_command_long_t mavcl;

                // decode SET_MODE message
                mavlink_msg_command_long_decode(&mavmsg, &mavcl);
                printf("%i, %f", mavcl.command, mavcl.param1);

                /* Send COMMAND_ACK */
                /* ARM */
                if (mavcl.command == MAV_CMD_COMPONENT_ARM_DISARM && mavcl.param1 == 1.0){ // CMD ID 400
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, MAV_CMD_COMPONENT_ARM_DISARM,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                    base_mode = ARDUPILOT_GUIDED_ARMED;
                }
                /* DISARM */
                if (mavcl.command == MAV_CMD_COMPONENT_ARM_DISARM && mavcl.param1 == 0.0){ // CMD ID 400
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, MAV_CMD_COMPONENT_ARM_DISARM,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                    base_mode = ARDUPILOT_GUIDED_DISARMED;
                    mission_start = false;
                }
                /* MAV_CMD_MISSION_START */
                if (mavcl.command == MAV_CMD_MISSION_START){ // CMD ID 300
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                    mission_start = true;
                }
                if (mavcl.command == MAV_CMD_REQUEST_PROTOCOL_VERSION){ // CMD ID 519
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                }
                break;
            }
            }
        }
        memset(buf, 0, BUFFER_LENGTH);
        usleep(1000); // Sleep 10 msec
        ros::spinOnce();
    }
}

uint64_t microsSinceEpoch(){
    struct timeval tv;
    uint64_t micros = 0;

    gettimeofday(&tv, NULL);  
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}
