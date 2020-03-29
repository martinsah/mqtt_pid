#include <iostream>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include "mqtt/async_client.h"

using namespace std;

const string SERVER_ADDRESS { "tcp://localhost:1883" };
const string CLIENT_ID      { "async_consume" };
const string TOPIC          { "adc/1" };

const int  QOS = 1;

class pid_controller {
    public:
    double kp,ki,kd,intcap,outcap_pos,outcap_neg,setpoint;
    double iterm, pterm, dterm;
    double last_error;
    
    pid_controller(){
        kp = 1;
        ki = 0;
        kd = 0;
        intcap = 0;
        outcap_pos = 1.0;
        outcap_neg = 0;
        setpoint = 1;
        iterm = 0;
        last_error = 0;
    }
    
    double operator()(double input){
        double rval;
        double error = input - setpoint;
        pterm = -error * kp;
        iterm += error * ki;
        dterm = kd * (last_error - error);
        last_error = error;
        if(iterm > intcap)
            iterm = intcap;
        if(-iterm > intcap)
            iterm = -intcap;
        rval = pterm + iterm + dterm;
        if(rval > outcap_pos)
            rval = outcap_pos;
        if(rval < outcap_neg)
            rval = outcap_neg;
        return rval;
    }
};


/////////////////////////////////////////////////////////////////////////////
int main(int ac, char **av)
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("server",  po::value<std::string>()->default_value("localhost"), "mqtt server")
        ("port",    po::value<int>()->default_value(1883),"mqtt port")

        ("pwmtopic",   po::value<std::string>()->default_value("pwm1"), "mqtt publish topic for pwm output")
        ("pwmtopictopic",   po::value<std::string>()->default_value("pid/pwmtopic"), "mqtt subscribe topic to dynamically change pwm output topic")
        
        ("sensor", po::value<std::string>()->default_value("adc/1"), "mqtt default subscribe topic for temperature sensor")
        ("sensortopic", po::value<std::string>()->default_value("sensortopic"), "mqtt (sensor topic) (subscribe topic) for temperature sensor e.g. dynamic control of temperature source data")
        ("setpointtopic", po::value<std::string>()->default_value("pid/set"), "mqtt setpoint topic")
        
        ("topic_kp", po::value<std::string>()->default_value("pid/kp"), "mqtt subscribe topic for online kp updates")
        ("topic_ki", po::value<std::string>()->default_value("pid/ki"), "mqtt subscribe topic for online ki updates")
        ("topic_kd", po::value<std::string>()->default_value("pid/kd"), "mqtt subscribe topic for online kd updates")
        
        ("td",  po::value<int>()->default_value(500), "time between iterations (integer milliseconds)")
        ("kp",   po::value<double>()->default_value(1.0) , "kp")
        ("ki",   po::value<double>()->default_value(.001), "ki")
        ("kd",   po::value<double>()->default_value(0.1) , "kd")
        ("intcap",   po::value<double>()->default_value(10) , "integral term cap (+-)")

        ("verbose", "report to stdout")
        ("csv", "csv readout to stdout")

    ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);    

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    bool verbose=false;
    if (vm.count("verbose")){
        std::cout << "verbose set\n";
        verbose = true;
    }
    
    bool csv=false;
    if (vm.count("verbose")){
        std::cout << "verbose set\n";
        verbose = true;
    }
   
    int port = vm["port"].as<int>();
    std::string server = vm["server"].as<std::string>();
    
    int td = vm["td"].as<int>();
    double kp = vm["kp"].as<double>();
    double ki = vm["ki"].as<double>();
    double kd = vm["kd"].as<double>();
    double intcap = vm["intcap"].as<double>();
    
    std::string pwmtopic = vm["pwmtopic"].as<std::string>();

    std::string sensortopic = vm["sensor"].as<std::string>();
    std::string sensortopictopic = vm["sensortopic"].as<std::string>();
    std::string pwmtopictopic = vm["pwmtopictopic"].as<std::string>();
    std::string setpointtopic = vm["setpointtopic"].as<std::string>();
    std::string topic_kp = vm["topic_kp"].as<std::string>();
    std::string topic_ki = vm["topic_ki"].as<std::string>();
    std::string topic_kd = vm["topic_kd"].as<std::string>();
       
    std::cout << "done parsing command line\n" << std::flush;
    std::cout << "\ntp= " << kp;
    std::cout << "\nti= " << ki;
    std::cout << "\ntd= " << kd;
    std::cout << "\nintcap= " << intcap;
    std::cout << "\npwmtopic= " << pwmtopic;
    std::cout << "\nsetpointtopic= " << setpointtopic;
    std::cout << "\nsensortopic= " << sensortopic;
    std::cout << "\nsensortopictopic= " << sensortopictopic;
    std::cout << "\npwmtopictopic= " << pwmtopictopic;
    std::cout << "\ntopic_kp= " << topic_kp;
    std::cout << "\ntopic_ki= " << topic_ki;
    std::cout << "\ntopic_kd= " << topic_kd;
    
    pid_controller pid{};
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.intcap = intcap;
    pid.outcap_neg = 0;
    pid.outcap_pos = 1.0;
    
    
    // mqtt
    const int    QOS = 1;
    const auto PERIOD = std::chrono::seconds(5);
    const int MAX_BUFFERED_MSGS = 120;  // 120 * 5sec => 10min off-line buffering
    const std::string PERSIST_DIR { "data-persist" };

    std::string address = std::string("tcp://") + server + std::string(":") + std::to_string(port);
    mqtt::async_client cli(address, "", MAX_BUFFERED_MSGS, PERSIST_DIR);
    
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(MAX_BUFFERED_MSGS * PERIOD);
    connOpts.set_clean_session(true);
    connOpts.set_automatic_reconnect(true);

    try {
        std::cout << "Connecting to server '" << address << "'..." << std::flush;
        cli.connect(connOpts)->wait();
        cli.start_consuming();
        std::cout << "OK\n" << std::endl;
    }
    catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
        return 1;
    }

    // mqtt publish topics
    shared_ptr<mqtt::topic> pwm;
    pwm = make_shared<mqtt::topic>(cli, pwmtopic, QOS, true);
    
    // mqtt consume topics
    cli.start_consuming();
    std::cout << "Subscribing to topic[s]\n";
    cli.subscribe(sensortopic, QOS)->wait();
    cli.subscribe(setpointtopic, QOS)->wait();
    cli.subscribe(sensortopictopic, QOS)->wait();
    cli.subscribe(pwmtopictopic, QOS)->wait();
    cli.subscribe(topic_kp, QOS)->wait();
    cli.subscribe(topic_ki, QOS)->wait();
    cli.subscribe(topic_kd, QOS)->wait();

    mqtt::const_message_ptr mqtt_msg;
    

    
    
    
    // Consume messages
    
    
    while (true) {
        double input_val = 0;
        double outval = 0;
        while(cli.try_consume_message(&mqtt_msg)){
            std::string topic = mqtt_msg->get_topic();
            std::string payload = mqtt_msg->to_string();
            //std::cout << "\n" << topic << ": " << payload;
            if(topic == sensortopic){
                input_val = std::stod(payload);
                continue;
            }
            if(topic == sensortopictopic){
                std::cout << "\nSwitching to sensor topic: " << payload;
                cli.unsubscribe(sensortopic);
                sensortopic = payload;
                cli.subscribe(sensortopic, QOS)->wait();
                continue;
            }            
            if(topic == pwmtopictopic){
                std::cout << "\nSwitching to PWM output topic: " << payload;
                pwm = make_shared<mqtt::topic>(cli, payload, QOS, true);
                continue;
            }
            if(topic == topic_kp){
                std::cout << "\nReceived kp update: " << payload;
                pid.kp = std::stod(payload);
                continue;
            }           
            if(topic == setpointtopic){
                std::cout << "\nReceived setpointtopic update: " << payload;
                pid.setpoint = std::stod(payload);
                continue;
            }
            if(topic == topic_ki){
                std::cout << "\nReceived ki update: " << payload;
                pid.ki = std::stod(payload);
                pid.iterm = 0;
                continue;
            }
            if(topic == topic_kd){
                std::cout << "\nReceived kd update: " << payload;
                pid.kd = std::stod(payload);
                continue;
            }
        }
        outval = pid(input_val);
        pwm->publish(std::to_string(outval));

        //
        this_thread::sleep_for(chrono::milliseconds(td));
    }

    // Disconnect

    cout << "\nShutting down and disconnecting from the MQTT server..." << flush;
    cli.unsubscribe(TOPIC)->wait();
    cli.stop_consuming();
    cli.disconnect()->wait();
    cout << "OK" << endl;


    return 0;
}
