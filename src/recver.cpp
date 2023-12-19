#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <cctype>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h> 
#include <signal.h>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "xwr_raw_ros/RadarFrameFull.h"

using namespace std;

class FrameBuffer{
    public:
        FrameBuffer(size_t _capacity, size_t _frame_size){
            buffer = (unsigned char*)calloc(_capacity, sizeof(char));
            capacity   = _capacity;
            frame_size = _frame_size;
            curr_idx   = 0;
            last_seqn  = 0;
        }

        void pad_zeros(size_t n_msgs, size_t msg_size){
            size_t total_size = n_msgs*msg_size;

            if (curr_idx + total_size > capacity){
                memset(buffer+curr_idx, 0, (capacity-curr_idx));
                memset(buffer, 0, ((curr_idx + total_size) % capacity));
                curr_idx = (curr_idx + total_size) % capacity;
            }
        }

        unsigned char* add_msg(int seqn, unsigned char* msg, size_t msg_len){
            unsigned char* new_frame = NULL;

            if (seqn > last_seqn + 1){
                printf("Packet drop.");
                pad_zeros((seqn - last_seqn - 1), msg_len);
            }

            last_seqn = seqn;
            
            if (curr_idx + msg_len > capacity){
                memcpy(buffer+curr_idx, msg, (capacity - curr_idx));
                memcpy(buffer, msg+(capacity-curr_idx), (msg_len - (capacity - curr_idx)));
            }
            else{
                memcpy(buffer+curr_idx, msg, msg_len);
            }

            size_t old_frame_idx = curr_idx / frame_size;
            curr_idx = (curr_idx + msg_len) % capacity;
            size_t new_frame_idx = curr_idx / frame_size;

            if (old_frame_idx != new_frame_idx){
                return buffer + old_frame_idx*frame_size;
            }
            else{
                return NULL;
            }
        }

        unsigned char* buffer;
        size_t capacity;
        size_t frame_size;
        size_t curr_idx;
        size_t last_seqn;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "xwr_radar_recver");

    cout.setf(ios::fixed,ios::floatfield);
    cout.precision(3);

    // Parse args.
    std::string host_ip              = "192.168.33.30";
    int host_data_port               = 4098;
    ros::NodeHandle nh_("~");
    nh_.getParam("host_ip",             host_ip);
    nh_.getParam("host_data_port",      host_data_port);

    // Get parameters of radar and publish topic.
    ros::NodeHandle nh;

    std::string       platform;
    int               adc_output_fmt;
    float             range_bias;
    vector<float>     rx_phase_bias;
    int               frame_size;

    float             chirp_time;
    float             chirp_slope;
    float             frame_time;
    float             velocity_max;
    float             velocity_res;

    int               sample_rate;
    float             range_max;
    float             range_res;

    vector<int>       rx;
    vector<int>       tx;
    int               n_chirps;
    int               n_rx;
    int               n_samples;
    vector<int>       shape;

    nh.getParam("radar_config/Platform", platform);
    nh.getParam("radar_params/adc_output_fmt", adc_output_fmt);
    nh.getParam("radar_params/range_bias", range_bias);
    nh.getParam("radar_params/rx_phase_bias", rx_phase_bias);
    nh.getParam("radar_params/frame_size", frame_size);

    nh.getParam("radar_params/chirp_time", chirp_time);
    nh.getParam("radar_params/chirp_slope", chirp_slope);
    nh.getParam("radar_params/frame_time", frame_time);
    nh.getParam("radar_params/velocity_max", velocity_max);
    nh.getParam("radar_params/velocity_res", velocity_res);

    nh.getParam("radar_params/sample_rate", sample_rate);
    nh.getParam("radar_params/range_max", range_max);
    nh.getParam("radar_params/range_res", range_res);

    nh.getParam("radar_params/rx", rx);
    nh.getParam("radar_params/tx", tx);
    nh.getParam("radar_params/n_chirps", n_chirps);
    nh.getParam("radar_params/n_rx", n_rx);
    nh.getParam("radar_params/n_samples", n_samples);
    shape = {n_chirps, n_rx, n_samples};

    cout << "platform: "       << platform << endl;
    cout << "adc_output_fmt: " << adc_output_fmt << endl;
    cout << "range_bias: "     << range_bias << endl;
    cout << "rx_phase_bias: ";
    for (auto x : rx_phase_bias){
         cout << x << ' ';
    }
    cout << endl;
    cout << "frame_size: "  << frame_size << endl;

    cout << "chirp_time: "   << chirp_time << endl;
    cout << "chirp_slope: "  << chirp_slope << endl;
    cout << "frame_time: "   << frame_time << endl;
    cout << "velocity_max: " << velocity_max << endl;
    cout << "velocity_res: " << velocity_res << endl;

    cout << "sample_rate: " << sample_rate << endl;
    cout << "range_max: "   << range_max << endl;
    cout << "range_res: "   << range_res << endl;

    cout << "rx: ";
    for (auto x : rx){
         cout << x << ' ';
    }
    cout << endl;
    cout << "tx: ";
    for (auto x : tx){
         cout << x << ' ';
    }
    cout << endl;
    cout << "n_chirps: " << n_chirps << endl;
    cout << "n_rx: " << n_rx << endl;
    cout << "n_samples: " << n_samples << endl;
    cout << "shape: ";
    for (auto x : shape){
         cout << x << ' ';
    }
    cout << endl;

    ros::Publisher pub = nh.advertise<xwr_raw_ros::RadarFrameFull>("radar_data", 1);

	// socket address used for the server
	struct sockaddr_in server_address;
	memset(&server_address, 0, sizeof(server_address));
	server_address.sin_family = AF_INET;
	server_address.sin_addr.s_addr = inet_addr(host_ip.c_str());
	server_address.sin_port = htons((unsigned short)host_data_port);

	// create a UDP socket, creation returns -1 on failure
	int sock;
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("could not create socket\n");
		return 1;
	}

	// bind it to listen to the incoming connections on the created server
	// address, will return -1 on error
	if ((bind(sock, (struct sockaddr *)&server_address,
	          sizeof(server_address))) < 0) {
		printf("could not bind socket\n");
		return 1;
	}

    int socksize = 131071;
    if ((setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &socksize, sizeof(int))) < 0)
    {
        printf("error setting sock opts\n");
        return 1;
    }

	// allocate frame buffer.
    FrameBuffer frame_buffer(2*frame_size, frame_size);
    
	// run indefinitely
    xwr_raw_ros::RadarFrameFull frame;
    frame.platform       = platform;
    frame.adc_output_fmt = adc_output_fmt;
    frame.range_bias     = range_bias;
    frame.rx_phase_bias  = rx_phase_bias;

    frame.chirp_time     = chirp_time;
    frame.chirp_slope    = chirp_slope;
    frame.frame_time     = frame_time;
    frame.velocity_max   = velocity_max;
    frame.velocity_res   = velocity_res;

    frame.sample_rate    = sample_rate;
    frame.range_max      = range_max;
    frame.range_res      = range_res;

    frame.tx             = vector<uint8_t>(tx.begin(), tx.end());
    frame.rx             = vector<uint8_t>(rx.begin(), rx.end());
    frame.shape          = vector<uint32_t>(shape.begin(), shape.end());

    unsigned int seqn = 0;
    unsigned int bytec = 0;
	while (ros::ok()) {
		unsigned char buffer[2048];

		// read content into buffer from an incoming client
        /* auto t1 = chrono::high_resolution_clock::now(); */

		int len = recvfrom(sock, buffer, sizeof(buffer), 0, NULL, NULL);

        seqn = (unsigned int) *buffer;
        bytec = (unsigned int) *(buffer + 4);

        unsigned char* frame_data = frame_buffer.add_msg(seqn, buffer+10, len-10);

        /* auto t2 = chrono::high_resolution_clock::now(); */
        /* chrono::duration<double, std::milli> ms_double = t2 - t1; */
        /* cout << ms_double.count() << "ms\n"; */

        if (frame_data){
            frame.data.assign((int16_t *)frame_data, (int16_t *)(frame_data + frame_buffer.frame_size));
            pub.publish(frame);
        } 

	}

	return 0;

}

